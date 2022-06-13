use las::{Read, Reader, Writer, Write, point::Classification, raw};
use std::{fs::File, fs::metadata, fs::create_dir, fs::read_dir, env, time, path::Path, ffi::OsStr};
//use std::thread;

/*fn multi_grid(file_path: &String, pos_x: usize, pos_y: usize, min_x: &f64, min_y: &f64, cell_size: &f64) -> (Vec<las::Point>, usize, usize) {
    let mut cell = Vec::new();
    let mut reader = Reader::from_path(file_path).unwrap();
    
    for wrapped_point in reader.points() {
        let point = wrapped_point.unwrap();
        if point.x >= (cell_size*pos_x as f64) + min_x && point.x < (cell_size*(pos_x+1) as f64) + min_x{
            if point.y >= (cell_size*pos_y as f64) + min_y && point.y < (cell_size*(pos_y+1) as f64) + min_y {
                cell.push(point.clone());
            }
        }       
    }
    (cell, pos_x, pos_y)
}

// Divide the point cloud into grid of size MxM cells
fn multi_grid_division(file_path: &String, cell_size: f64) -> Vec<Vec<Vec<las::Point>>>{
    let mut threads = vec![];
    let reader = Reader::from_path(file_path).unwrap();

    let min_x = reader.header().bounds().min.x;
    let min_y = reader.header().bounds().min.y;
    let max_x = reader.header().bounds().max.x;
    let divisions_x = cell_size;

    let grid_size = ((max_x - min_x)/divisions_x).ceil() as usize;

    let mut grids:Vec<Vec<Vec<las::Point>>> = Vec::new();
    for i in 0..grid_size{
        grids.push(Vec::new());
        for _ in 0..grid_size{
            grids[i].push(Vec::new());
        }
    }

    for i in 0..grid_size {
        for j in 0..grid_size {
            let file_path = file_path.clone();
            threads.push(thread::spawn(move || 
                multi_grid(&file_path, i, j, &min_x, &min_y, &cell_size)
            ));
        }
    }
    for thread in threads {
        let (cell, pos_x, pos_y) = thread.join().unwrap();
        grids[pos_x][pos_y] = cell;
    }
    grids
}*/

fn grid_division(las_file: &mut las::Reader, cell_size: f64) -> Vec<Vec<Vec<las::Point>>>{
    let min_x = las_file.header().bounds().min.x;
    let min_y = las_file.header().bounds().min.y;
    let max_x = las_file.header().bounds().max.x;

    let divisions_x = cell_size;
    let divisions_y = cell_size;

    let grid_size = ((max_x - min_x)/divisions_x).ceil() as usize;

    let points_iter = las_file.points();
    
    let mut grids:Vec<Vec<Vec<las::Point>>> = Vec::new();
    for i in 0..grid_size{
        grids.push(Vec::new());
        for _ in 0..grid_size{
            grids[i].push(Vec::new());
        }
    }

    'outer: for wrapped_point in points_iter {
        let point = wrapped_point.unwrap();
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

fn filter_height_density(point_cell: &Vec<las::Point>, min_density: usize, empty:usize , non_empty:usize , end: usize) -> Vec<las::Point> {
    let mut cell = Vec::new();
    let mut candidate = false;
    let mut count_non_empty = 0;
    let mut count_empty = 0;
    
    let max_z = point_cell.iter().map(|p| p.z).fold(f64::NAN, f64::max);
    let min_z = point_cell.iter().map(|p| p.z).fold(f64::NAN, f64::min);
    for i in 0..end {
        if (max_z - (i as f64)) > min_z {
            let density = point_cell.iter().filter
                    (|p| p.z <= max_z - (i as f64) && p.z > max_z - (i as f64 + 1.)).count();
            
            if density > min_density {
                count_non_empty += 1;
                count_empty = 0;
            
            } else {
                count_empty += 1;
                count_non_empty = 0;
            }
            
            if count_non_empty > non_empty {
                break;
            }
            
            if count_empty > empty {
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
fn filter_cells(point_cloud: &Vec<Vec<Vec<las::Point>>>, min_density: usize, empty:usize , non_empty:usize , end: usize) -> Vec<Vec<Vec<las::Point>>> {   
    let mut filtered_point_cloud = Vec::new();
    for i in 0..point_cloud.len(){
        filtered_point_cloud.push(Vec::new());
        for j in 0..point_cloud[i].len(){
            filtered_point_cloud[i].push(filter_height_density(&point_cloud[i][j], min_density, empty, non_empty, end));
        }
    }
    filtered_point_cloud
}

//-----------------------------------------------PADDING MATRIX----------------------------------------------
fn empty_vector(num: usize) -> Vec<Vec<las::Point>> {
    let mut vec: Vec<Vec<las::Point>>   = Vec::new();
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
fn erode_neighborhood(point_cloud: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>, neighbourhood: usize) -> Vec<Vec<Vec<las::Point>>> {
    let padding = 
        match neighbourhood {
            8 => {
                1
            },
            16 => {
                2
            }
            24 => {
                3
            }
            _ => {
                panic!("Invalid neighbourhood size");
            }
        };

    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<Vec<las::Point>>> = create_padded_point_cloud(&eroded, padding);

    for i in padding..padded.len() - padding {
        for j in padding..padded[i].len() - padding {
            let mut has_neighbour = false;
            if padded[i][j].len() > 0 {
                let min_neighbours = 
                    if i == 1 || j == 1 || i == (padded.len() - 2) || j == (padded[i].len() - 2) || neighbourhood != 8 { //Border cells
                        1
                    }
                    else {
                        2
                    };
                for k in j-padding..j+padding+1 {
                    if ((padded[i-padding][k].len() > 0) as i32 
                        + (padded[i+padding][k].len() > 0) as i32 
                        ) >= min_neighbours {
                            has_neighbour = true;
                            break;
                    }
                }
                if !has_neighbour {
                    for k in i-padding..i+padding+1 {
                        if ((padded[k][j-padding].len() > 0) as i32 
                            + (padded[k][j+padding].len() > 0) as i32 
                            ) >= 1 {
                                has_neighbour = true;
                                break;
                        }
                    }
                }  
                match has_neighbour {
                    true => {
                        eroded[i-padding][j-padding] = 
                            original[i-padding][j-padding].clone();
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
        4 => {
            return erode_4(filtered, original);
        },
        8 | 16 | 24 => {
            return erode_neighborhood(filtered, original, neighbourhood);
        },
        _ => {
            panic!("Invalid neighbourhood size");
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
                    dilated[i-1][j-1] = original[i-1][j-1].clone();
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
                    if point.classification != Classification::Ground {
                        writer.write(point.clone()).unwrap();
                    }
                }
            }
        }
    }
    writer.close().unwrap();
}

fn write_csv(point_cloud: &Vec<Vec<Vec<las::Point>>>, output: &String) {
    let mut writer = csv::Writer::from_path(output).unwrap();
    for i in 0..point_cloud.len(){
        for j in 0..point_cloud[i].len(){
            if point_cloud[i][j].len() > 0{
                for point in &point_cloud[i][j]{
                    writer.write_record(&[format!("{}", point.x), format!("{}", point.y), format!("{}", point.z)]).unwrap();
                }
            }
        }
    }
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
   
    //Grid Division
    let now = time::Instant::now();
    let gridded = grid_division(&mut reader, 12.);
    println!("Grid division time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Filtering
    let now = time::Instant::now();
    let filtered = filter_cells(&gridded, 6, 2, 7, 20);
    println!("Filtering time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Erosions
    let now = time::Instant::now();
    let eroded4 = erode(&filtered, &gridded, 4);
    let eroded8 = erode(&eroded4, &gridded, 8);
    let eroded16 = erode(&eroded8, &gridded, 16);
    let eroded24 = erode(&eroded16, &gridded, 24);
    println!("Erosion time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Dilation
    let now = time::Instant::now();
    let dilated = dilate(&eroded24, &gridded);
    println!("Dilation time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Writing las/laz and csv
    let now = time::Instant::now();
    write_las(&eroded16, raw_header, output);
    println!("Writing time: {:?} millisecs.", now.elapsed().as_millis());
    write_csv(&dilated, &format!("{}csv",&output[0..output.len()-3]));

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
        
        } else {
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