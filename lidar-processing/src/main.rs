use las::{Read, Reader, Writer, Write, point::Classification, raw, Header};
use std::{fs::File, fs::metadata, fs::create_dir, fs::read_dir, env, time, path::Path, ffi::OsStr};
use rand::{thread_rng, seq::SliceRandom};

fn reduce_ground(point_cloud: &mut las::Reader, ground_percentage: f64) -> Vec<las::Point> {
    let mut points_vector = Vec::new();
    let remove = [(true, 1.-ground_percentage), (false, ground_percentage)];

    for point in point_cloud.points() {
        let point = point.unwrap();
        if point.classification == Classification::Ground {
            let mut rng = thread_rng();
            if !remove.choose_weighted(&mut rng, |item| item.1).unwrap().0 {
                points_vector.push(point);
            }
        }
        else{
            points_vector.push(point);
        }
    }
    points_vector
}

fn grid_division(file_header: &Header, point_cloud: Vec<las::Point>,cell_size: f64) -> Vec<Vec<Vec<las::Point>>>{
    let min_x = file_header.bounds().min.x;
    let min_y = file_header.bounds().min.y;
    let max_x = file_header.bounds().max.x;

    let divisions_x = cell_size;
    let divisions_y = cell_size;

    let grid_size = ((max_x - min_x)/divisions_x).ceil() as usize;
    
    let mut grids:Vec<Vec<Vec<las::Point>>> = Vec::new();
    for i in 0..grid_size{
        grids.push(Vec::new());
        for _ in 0..grid_size{
            grids[i].push(Vec::new());
        }
    }

    'outer: for point in point_cloud {
        for i in 0..grid_size {
            if point.x >= (divisions_x*i as f64) + min_x && point.x < (divisions_x*(i+1) as f64) + min_x{
                for j in 0..grid_size{
                    if point.y >= (divisions_y*j as f64) + min_y && point.y < (divisions_y*(j+1) as f64) + min_y {
                        grids[i][j].push(point);
                        continue 'outer;
                    }
                }
            } 
        }
    }
    grids
}
//-------------------------------------------NOISE REMOVAL-------------------------------------------------

//Euclidean distance between two points
fn euclidean_distance(point1: &las::Point, point2: &las::Point) -> f64 {
    let x_diff = point1.x - point2.x;
    let y_diff = point1.y - point2.y;
    let z_diff = point1.z - point2.z;
    (x_diff*x_diff + y_diff*y_diff + z_diff*z_diff).sqrt()
}

fn mean_zvalue(points: &Vec<las::Point>) -> f64 {
    let mut sum = 0.0;
    for point in points {
        sum += point.z;
    }
    sum / points.len() as f64
}

//Check if point is noise
fn is_noise(point: &las::Point, cell: &Vec<las::Point>, epsilon: f64, limit: usize, diff_threshold: f64) -> bool {
    let mut count = 0;
    let mut noise = true;
    let mean_z = mean_zvalue(&cell);
    for i in 0..cell.len() {
        if &cell[i] != point && (euclidean_distance(point, &cell[i]) < epsilon) {
            count += 1;
            if count >= limit && (point.z - mean_z).abs() < diff_threshold {
                noise = false;
                break;
            }
        }
    }
    noise
}

fn clean_noise(point_cloud: &Vec<Vec<Vec<las::Point>>>, up_epsilon: f64, up_limit: usize, diff_threshold: f64) -> Vec<Vec<Vec<las::Point>>> {
    let mut cleaned = point_cloud.clone();
    for i in 0..cleaned.len() {
        for j in 0..cleaned[i].len() {
            loop {
                if cleaned[i][j].len() != 0 {
                    let max_z = cleaned[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::max);
                    let highest_point = cleaned[i][j].iter().find(|p| p.z == max_z).unwrap();
                    if is_noise(&highest_point, &cleaned[i][j], up_epsilon, up_limit, diff_threshold) {
                        let index = cleaned[i][j].iter().position(|p| p == highest_point).unwrap();
                        cleaned[i][j].remove(index);
                    }
                    else {
                        break;
                    }
                }
                else {
                    break;
                }
            }
        }
    }
    cleaned
}
//-------------------------------------------FILTER HEIGHT DENSITYS-------------------------------------------------

fn filter_height_density(point_cell: &Vec<las::Point>, min_density: usize, empty:usize , non_empty_cont:usize, non_empty: usize, end: usize) -> Vec<las::Point> {
    let mut cell = Vec::new();
    let mut candidate = false;
    let mut count_non_empty = 0;
    let mut count_empty = 0;
    let mut total_count_non_empty = 0;
    
    let max_z = point_cell.iter().map(|p| p.z).fold(f64::NAN, f64::max);
    let min_z = point_cell.iter().map(|p| p.z).fold(f64::NAN, f64::min);
    for i in 0..end {
        if (max_z - (i as f64)) > min_z{
            let density = point_cell.iter().filter
                    (|p| p.z <= max_z - (i as f64) && p.z > max_z - (i as f64 + 1.)).count();
            
            if density > min_density {
                count_non_empty += 1;
                total_count_non_empty += 1;
                count_empty = 0;
            
            } else {
                count_empty += 1;
                count_non_empty = 0;
            }

            if count_non_empty >= non_empty_cont || total_count_non_empty >= non_empty {
                break;
            }

            if count_empty >= empty {
                candidate = true;
                break;
            }
        }
        
        else {
            break;
        }
    }

    if candidate {
        for point in point_cell.iter().filter(|p| p.z > max_z - 20.){
            cell.push(point.clone());
        }
    }
    cell
}

//Filter all cells
fn filter_cells(point_cloud: &Vec<Vec<Vec<las::Point>>>, min_density: usize, empty:usize , non_empty_cont: usize, non_empty:usize, end: usize) -> Vec<Vec<Vec<las::Point>>> {   
    let mut filtered_point_cloud = Vec::new();
    for i in 0..point_cloud.len(){
        filtered_point_cloud.push(Vec::new());
        for j in 0..point_cloud[i].len(){
            filtered_point_cloud[i].push(filter_height_density(&point_cloud[i][j], min_density, empty, non_empty_cont, non_empty, end));
        }
    }
    filtered_point_cloud
}

//-----------------------------------------------PADDING MATRIX----------------------------------------------
fn empty_vector(num: usize) -> Vec<Vec<las::Point>> {
    let mut vec: Vec<Vec<las::Point>> = Vec::new();
    for _ in 0..num {
        vec.push(Vec::new());
    }
    vec
}

fn create_padded_point_cloud(point_cloud: &Vec<Vec<Vec<las::Point>>>, padding: usize) -> Vec<Vec<Vec<las::Point>>> {
    let mut padded_point_cloud: Vec<Vec<Vec<las::Point>>> = Vec::new();
    let empty_cells: Vec<Vec<las::Point>> = empty_vector(padding);

    for i in 0..padding { //Add padding rows to the top
        padded_point_cloud.push(Vec::new());
        for _ in 0..(point_cloud[i].len() + padding*2) {
            padded_point_cloud[i].push(Vec::new());
        }
    }

    for i in 0..point_cloud.len(){
        padded_point_cloud.push(Vec::new()); //Create new row
        for j in 0..empty_cells.len(){
            padded_point_cloud[i+padding].push(empty_cells[j].clone()); //Add padding cells to the left
        }
        for j in 0..point_cloud[i].len(){
            padded_point_cloud[i+padding].push(point_cloud[i][j].clone());
        }
        for j in 0..empty_cells.len(){
            padded_point_cloud[i+padding].push(empty_cells[j].clone()); //Add padding cells to the right
        }
    }

    for i in padding..padding*2 { //Add padding rows to the bottom
        padded_point_cloud.push(Vec::new());
        for _ in 0..(point_cloud[i].len() + padding*2) {
            padded_point_cloud[point_cloud.len()+i].push(Vec::new());
        }
    }
    padded_point_cloud
}

//-----------------------------------------------EROSIONS----------------------------------------------
fn erode_4(point_cloud: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<Vec<las::Point>>> = create_padded_point_cloud(&eroded, 1);
    
    for i in 1..padded.len() - 1 {
        for j in 1..padded[i].len() - 1 {
            // At least 1 neighbour in the neighbourhood (top, bottom, right, left)
            if  padded[i][j].len() > 0 {
                let min_neighbours = 
                    if i == 1 || j == 1 || i == (padded.len() - 2) || j == (padded[i].len() - 2) { //Border cells
                        1
                    }
                    else {
                        2
                    };

                if ( ((padded[i+1][j].len() > 0) as i32) + ((padded[i-1][j].len() > 0) as i32) 
                    + ((padded[i][j+1].len() > 0) as i32) + ((padded[i][j-1].len() > 0) as i32) 
                    ) >= min_neighbours  {
                        eroded[i-1][j-1] = original[i-1][j-1].clone();
                }

                else {
                    eroded[i-1][j-1].clear();
                }
            }
        }
    }
    eroded
}

fn points_under_max(cell: &Vec<las::Point>, meters: f64) -> Vec<las::Point> {
    let mut point_cell = Vec::new();
    let max_z = cell.iter().map(|p| p.z).fold(f64::NAN, f64::max);

    for point in cell {
        if point.z >= max_z - meters {
            point_cell.push(point.clone());
        }
    }
    point_cell
}

fn erode_neighborhood(point_cloud: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>, padding: usize) -> Vec<Vec<Vec<las::Point>>> {
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<Vec<las::Point>>> = create_padded_point_cloud(&eroded, padding);

    for i in padding..padded.len() - padding {
        for j in padding..padded[i].len() - padding {
            let mut neighbours_per_level = Vec::new();
            for _ in 0..padding {
                neighbours_per_level.push(0);
            }
            
            let mut has_neighbour = false;
            
            if padded[i][j].len() > 0 {
                let border_cell = 
                if i == padding || j == padding || i == (padded.len() - (padding + 1) ) || j == (padded[i].len() - (padding + 1) ) { //Border cells
                    true
                }
                else {
                    false
                };

                for k in i-padding..i {
                    for l in j-padding..j+padding+1 {
                        if padded[k][l].len() > 0 {
                            neighbours_per_level[k-(i-padding)] += 1;
                        }
                        if padded[i+padding-(i-k)][l].len() > 0 {
                            neighbours_per_level[k-(i-padding)] += 1;
                        }
                    }
                }

                for x in 0..neighbours_per_level.len() {
                    if border_cell && neighbours_per_level[x] >= 1 {
                        has_neighbour = true;
                    }
                    else if !border_cell && i-x >= padding && neighbours_per_level[x] >= 1 {
                        has_neighbour = true;
                    }
                    else if !border_cell && neighbours_per_level[x] >= 2 {
                        has_neighbour = true;
                    }
                    else {
                        has_neighbour = false;
                        break;
                    }
                }

                
                
                if !has_neighbour {
                    for l in j-padding..j {
                        if padded[i][l].len() > 0 {
                            neighbours_per_level[l-(j-padding)] += 1;
                        }
                        if padded[i][j+padding-(j-l)].len() > 0 {
                            neighbours_per_level[l-(j-padding)] += 1;
                        }
                    }
                    
                    for x in 0..neighbours_per_level.len() {
                        if border_cell && neighbours_per_level[x] >= 1 {
                            has_neighbour = true;
                        }
                        else if !border_cell && i-x >= padding && neighbours_per_level[x] >= 1 {
                            has_neighbour = true;
                        }
                        else if !border_cell && neighbours_per_level[x] >= 2 {
                            has_neighbour = true;
                        }
                        else {
                            has_neighbour = false;
                            break;
                        }
                    }
                }
                
                match has_neighbour {
                    true => {
                        eroded[i-padding][j-padding] = 
                            points_under_max(&original[i-padding][j-padding], 23.);
                    },
                    false => {
                        eroded[i-padding][j-padding].clear();
                    }
                }
            }
        }
    }
    eroded
}

fn erode(filtered: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>, neighbourhood: usize) -> Vec<Vec<Vec<las::Point>>> {

    match neighbourhood {
        0 => {
            return erode_4(filtered, original);
        },
        _ => {
            return erode_neighborhood(filtered, original, neighbourhood);
        }
    }
}

//-----------------------------------------------DILATION----------------------------------------------
fn dilate(filtered: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
    let mut dilated = filtered.clone();
    let padded: Vec<Vec<Vec<las::Point>>> = create_padded_point_cloud(&dilated, 1);
    
    for i in 1..padded.len() - 1{
        for j in 1..padded[i].len() - 1 {
            let mut has_neighbour = false;
            let min_neighbours = 
                if i == 1 || j == 1 || i == (padded.len() - 2) || j == (padded[i].len() - 2) { //Border cells
                    1
                }
                else {
                    2
                };

            for k in j-1..j+2 {
                if ((padded[i-1][k].len() > 0) as i32 
                    + (padded[i+1][k].len() > 0) as i32 
                ) >= min_neighbours {
                    has_neighbour = true;
                    break;
                }
            }
            
            if !has_neighbour {
                if ((padded[i][j-1].len() > 0) as i32 
                    + (padded[i][j+1].len() > 0) as i32 
                ) >= min_neighbours {
                    has_neighbour = true;
                }
            }  

            match has_neighbour {
                true => {
                    dilated[i-1][j-1] = points_under_max(&original[i-1][j-1], 23.);
                },
                false => {
                    dilated[i-1][j-1].clear();
                }
            }
        }
    }
    dilated
}
//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------

fn convert_to_binary(point_cloud: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<bool>> {
    let mut binary = Vec::new();
    for i in 0..point_cloud.len() {
        binary.push(Vec::new());
        for _ in 0..point_cloud[i].len() {
            binary[i].push(false);
        }
    }
    
    for i in 0..binary.len() {
        for j in 0..binary[i].len() {
            if point_cloud[i][j].len() > 0 {
                binary[i][j] = true;
            }
            else {
                binary[i][j] = false;
            }
        }
    }
    binary
}


//------------------------------------BREADTH-FIRST-----------------------------------------
//-----------------------------------------------------------------------------------------

fn is_safe(matrix: &Vec<Vec<bool>>, visited: &Vec<Vec<bool>>, i: i32, j: i32) -> bool {
    if i < 0 || j < 0 || i >= matrix.len() as i32 
      || j >= matrix[i as usize].len() as i32
      || visited[i as usize][j as usize] 
      || !matrix[i as usize][j as usize] {
        return false;
    } else {
        return true;
    }
}

fn find_component_size(matrix: &Vec<Vec<bool>>) -> Vec<(usize, usize)> {
    let dx:[i32; 8] = [1, 1, 1, 0, -1, -1, -1, 0];
    let dy:[i32; 8]  = [1, 0, -1, -1, -1, 0, 1, 1];
    let mut candidates = Vec::new();
    
    let mut visited = Vec::new();
    for i in 0..matrix.len() {
        visited.push(Vec::new());
        for _ in 0..matrix[i].len() {
            visited[i].push(false);
        }
    }

    for i in 0..matrix.len() {
        for j in 0..matrix[i].len() {
            let mut count = 0;
            let mut cells_group = Vec::new();
            if !visited[i][j] && matrix[i][j] {
                let mut queue = Vec::new();
                queue.push((i, j));
                visited[i][j] = true;
                while queue.len() != 0 {
                    let (x, y) = queue.pop().unwrap();
                    cells_group.push((x, y));
                    count += 1;
                    for k in 0..8 {
                        let (x1, y1) = (x as i32 + dx[k], y as i32 + dy[k]);
                        if is_safe(matrix, &visited, x1, y1) {
                            queue.push((x1 as usize, y1 as usize));
                            visited[x1 as usize][y1 as usize] = true;
                        }
                    }
                }
                if count > 30 && maximum_distant_cells(&cells_group){
                    candidates.append(&mut cells_group);
                }
            }
        }
    }
    candidates
}

fn euclidean_cells(cell1: (usize, usize), cell2: (usize, usize)) -> f64 {
    let (x1, y1) = cell1;
    let (x2, y2) = cell2;
    let x_diff = (x1 as f64 - x2 as f64).powi(2);
    let y_diff = (y1 as f64 - y2 as f64).powi(2);
    (x_diff + y_diff).sqrt()
}

fn maximum_distant_cells(cells: &Vec<(usize, usize)>) -> bool{
    let mut result = false;
    let mut max_distance = 0.;
    
    for i in 0..cells.len()-1 {
        for j in i+1..cells.len() {
            if euclidean_cells(cells[i], cells[j]) > max_distance {
                max_distance = euclidean_cells(cells[i], cells[j]);
            }
        }
    }
    if max_distance > 18. {
        result = true;
    }
    result
}

fn create_result(cells_list: &Vec<(usize, usize)>, point_cloud: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
    let mut result = Vec::new();
    for i in 0..point_cloud.len() {
        result.push(Vec::new());
        for _ in 0..point_cloud[i].len() {
            result[i].push(Vec::new());
        }
    }
    for i in 0..cells_list.len() {
        result[cells_list[i].0][cells_list[i].1] = point_cloud[cells_list[i].0][cells_list[i].1].clone();
    }
    result
}

//-----------------------------------------------------------------------------------------
//---------------------------------------WRITING-------------------------------------------

//Writes into file all point cloud
fn write_las(point_cloud: &Vec<Vec<Vec<las::Point>>>, raw_header: raw::Header, output: &String) {
    let mut writer = Writer::from_path(output, las::Header::from_raw(raw_header).unwrap()).unwrap();

    for i in 0..point_cloud.len(){
        for j in 0..point_cloud[i].len(){
            if point_cloud[i][j].len() > 0{
                for point in &point_cloud[i][j]{
                    let mut point = point.clone();
                    if point.return_number > 5 {
                        point.return_number = 5;
                    }
                    //point.z = 0.;
                    if point.classification != Classification::Ground {
                        writer.write(point.clone()).unwrap();
                    }
                }
            }
        }
    }
    writer.close().unwrap();
}

//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------

fn execute_algorithms(input: &String, output: &String) {
    let mut reader = match Reader::from_path(input) {
        Ok(reader) => reader,
        Err(err) => {
            println!("{:?}", err);
            return;
        }
    };

    let mut file = File::open(input).unwrap();
    let raw_header = raw::Header::read_from(&mut file).unwrap(); //Leemos el header original para quedarnos con los datos de version, padding, scales...

    println!("----------Processing file: {}----------", input);

    //Ground Reduction
    let now = time::Instant::now();
    let ground_reduced = reduce_ground(&mut reader, 0.01);
    println!("Ground Reduction time: {:?} millisecs.", now.elapsed().as_millis());

    //Grid Division
    let now = time::Instant::now();
    let gridded = grid_division(reader.header(), ground_reduced, 7.5);
    println!("Grid division time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Filtering
    let now = time::Instant::now();
    let filtered = filter_cells(&gridded, 6, 4, 4, 7, 30);
    println!("Filtering time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Erosions
    let eroded4 = erode(&filtered, &gridded, 0);
    let eroded32 = erode(&eroded4, &gridded, 5);
    println!("Erosion time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Dilation
    let now = time::Instant::now();
    let dilated = dilate(&eroded32, &gridded);

    let eroded4 = erode(&dilated, &gridded, 0);
    let eroded32 = erode(&eroded4, &gridded, 4);
    println!("Dilation time: {:?} millisecs.", now.elapsed().as_millis());

    let binary = convert_to_binary(&eroded32);
    let candidates = find_component_size(&binary);
    let result = create_result(&candidates, &eroded32);
   
    //Writing las/laz and csv
    let now = time::Instant::now();
    write_las(&result, raw_header, output);
    println!("Writing time: {:?} millisecs.", now.elapsed().as_millis());

}

fn main() {
    let total_time = time::Instant::now();
    let args: Vec<String> = env::args().collect();
    let input = &args[1];
    let output = &args[2];
    let mut num_cells = 0;

    let md_input = metadata(input).unwrap();

    if md_input.is_dir() {
        let paths = read_dir(input).unwrap();
        if !(Path::new(output).exists()) {
            create_dir(output).unwrap();
        }
        
        for path in paths {
            let path = path.unwrap().path();
            let extension = path
                    .extension()
                    .and_then(OsStr::to_str);
            if extension == Some("las") || extension == Some("laz") {
                let filename = path.file_name().unwrap().to_str().unwrap();
                let input_path = format!("{}/{}", input, filename);
                let output_filename = format!("{}/FILTERED_{}.{}", output, &filename[0..filename.len()-4], extension.unwrap());
                num_cells += 1;
                execute_algorithms(&input_path, &output_filename);
            }
        }
        
    } else {
        if !(Path::new(output).exists()) {
            create_dir(output).unwrap();
        }

        let filename = Path::new(&input).file_name().unwrap().to_str().unwrap();
        let extension = Path::new(&input)
                        .extension()
                        .and_then(OsStr::to_str);
        let output_filename = format!("{}/FILTERED_{}.{}", output, &filename[0..filename.len()-4], extension.unwrap());
        num_cells += 1;
        execute_algorithms(input, &output_filename);
    }
    println!("Total time for {:?} cells: {:?} secs.", num_cells, total_time.elapsed().as_secs());
}