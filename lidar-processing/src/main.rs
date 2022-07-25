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
fn empty_vector(num: usize) -> Vec<bool> {
    let mut vec: Vec<bool> = Vec::new();
    for _ in 0..num {
        vec.push(false);
    }
    vec
}

fn create_padded_point_cloud(point_cloud: &Vec<Vec<bool>>, padding: usize) -> Vec<Vec<bool>> {
    let mut padded_point_cloud: Vec<Vec<bool>> = Vec::new();
    let empty_cells: Vec<bool> = empty_vector(padding);

    for i in 0..padding { //Add padding rows to the top
        padded_point_cloud.push(Vec::new());
        for _ in 0..(point_cloud[i].len() + padding*2) {
            padded_point_cloud[i].push(false);
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
            padded_point_cloud[point_cloud.len()+i].push(false);
        }
    }
    padded_point_cloud
}

//-----------------------------------------------EROSIONS----------------------------------------------
fn erode_4(point_cloud: &Vec<Vec<bool>>) -> Vec<Vec<bool>> {
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<bool>> = create_padded_point_cloud(&eroded, 1);
    
    for i in 1..padded.len() - 1 {
        for j in 1..padded[i].len() - 1 {
            // At least 1 neighbour in the neighbourhood (top, bottom, right, left)
            if  padded[i][j] == true {
                let min_neighbours = 
                    if i == 1 || j == 1 || i == (padded.len() - 2) || j == (padded[i].len() - 2) { //Border cells
                        1
                    }
                    else {
                        2
                    };

                if ( ((padded[i+1][j] == true) as i32) + ((padded[i-1][j] == true) as i32) 
                    + ((padded[i][j+1] == true) as i32) + ((padded[i][j-1] == true) as i32) 
                    ) >= min_neighbours  {
                        eroded[i-1][j-1] = true;
                }

                else {
                    eroded[i-1][j-1] = false;
                }
            }
        }
    }
    eroded
}

fn points_under_max(point_cloud: &Vec<Vec<Vec<las::Point>>>, meters: f64) -> Vec<Vec<Vec<las::Point>>> {
    let mut output = Vec::new();
    for i in 0..point_cloud.len() {
        output.push(Vec::new());
        for j in 0..point_cloud[i].len() {
            let cell = &point_cloud[i][j];
            let mut point_cell = Vec::new();

            let max_z = cell.iter().map(|p| p.z).fold(f64::NAN, f64::max);

            for point in cell {
                if point.z >= max_z - meters {
                    point_cell.push(point.clone());
                }
            }
            output[i].push(point_cell);
        }
    }
    output
}

fn erode_neighborhood(point_cloud: &Vec<Vec<bool>>, padding: usize) -> Vec<Vec<bool>> {
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<bool>> = create_padded_point_cloud(&eroded, padding);

    for i in padding..padded.len() - padding {
        for j in padding..padded[i].len() - padding {
            let mut neighbours_per_level = Vec::new();
            for _ in 0..padding {
                neighbours_per_level.push(0);
            }
            
            let mut has_neighbour = false;
            
            if padded[i][j] == true {
                let border_cell = 
                if i == padding || j == padding || i == (padded.len() - (padding + 1) ) || j == (padded[i].len() - (padding + 1) ) { //Border cells
                    true
                }
                else {
                    false
                };

                for k in i-padding..i {
                    for l in j-padding..j+padding+1 {
                        if padded[k][l] == true  {
                            neighbours_per_level[k-(i-padding)] += 1;
                        }
                        if padded[i+padding-(i-k)][l] == true {
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
                        if padded[i][l] == true  {
                            neighbours_per_level[l-(j-padding)] += 1;
                        }
                        if padded[i][j+padding-(j-l)] == true  {
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
                        eroded[i-padding][j-padding] = true
                    },
                    false => {
                        eroded[i-padding][j-padding] = false ;
                    }
                }
            }
        }
    }
    eroded
}

fn erode(filtered: &Vec<Vec<bool>>, neighbourhood: usize) -> Vec<Vec<bool>> {

    match neighbourhood {
        0 => {
            return erode_4(filtered);
        },
        _ => {
            return erode_neighborhood(filtered, neighbourhood);
        }
    }
}

//-----------------------------------------------DILATION----------------------------------------------
fn dilate(filtered: &Vec<Vec<bool>>) -> Vec<Vec<bool>> {
    let mut dilated = filtered.clone();
    let padded: Vec<Vec<bool>> = create_padded_point_cloud(&dilated, 1);
    
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
                if ((padded[i-1][k] == true ) as i32 
                    + (padded[i+1][k] == true ) as i32 
                ) >= min_neighbours {
                    has_neighbour = true;
                    break;
                }
            }
            
            if !has_neighbour {
                if ((padded[i][j-1] == true ) as i32 
                    + (padded[i][j+1] == true ) as i32 
                ) >= min_neighbours {
                    has_neighbour = true;
                }
            }  

            match has_neighbour {
                true => {
                    dilated[i-1][j-1] = true;
                },
                false => {
                    dilated[i-1][j-1] = false;
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

fn find_component_size(matrix: &Vec<Vec<bool>>, dist_thres:f64) -> Vec<(usize, usize)> {
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
                let ([(i1, j1), (i2, j2)], distance) = most_distant_cells(&cells_group);

                if (count > 30 && distance > dist_thres) || ((i1 == 0 || i1 == matrix.len()-1 || j1 == 0 || j1 == matrix[i1].len()-1) 
                                                            && i2 == 0 || i2 == matrix.len()-1 || j2 == 0 || j2 == matrix[i2].len()-1) {
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

fn most_distant_cells(cells: &Vec<(usize, usize)>) -> ([(usize, usize); 2], f64){
    let mut max_distance = 0.;
    let mut most_distant:[(usize, usize); 2] = [(0, 0), (0, 0)];
    
    for i in 0..cells.len()-1 {
        for j in i+1..cells.len() {
            if euclidean_cells(cells[i], cells[j]) > max_distance {
                max_distance = euclidean_cells(cells[i], cells[j]);
                most_distant = [cells[i], cells[j]];
            }
        }
    }

    (most_distant, max_distance)
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

    let binary = convert_to_binary(&filtered);
   
    //Erosions
    let eroded4 = erode(&binary,0);
    let eroded32 = erode(&eroded4,5);
    println!("Erosion time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Dilation
    let now = time::Instant::now();
    let dilated = dilate(&eroded32);

    let eroded4 = erode(&dilated, 0);
    let eroded32 = erode(&eroded4, 4);
    println!("Dilation time: {:?} millisecs.", now.elapsed().as_millis());

    
    let candidates = find_component_size(&eroded32, 18.);
    let result = create_result(&candidates, &gridded);

    let out = points_under_max(&result, 18.);
   
    //Writing las/laz and csv
    let now = time::Instant::now();
    write_las(&out, raw_header, output);
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