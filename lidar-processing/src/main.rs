use las::reader::PointIterator;
use las::{Read, Reader, Writer, Write, point::Classification, raw};
use std::{fs::File, fs::metadata, fs::create_dir, fs::read_dir, env, time, path::Path, ffi::OsStr};
use std::thread;
use std::borrow::Borrow;

/*fn thread_grid(las_reader: &mut las::Reader, pos_x: usize, pos_y: usize, min_x: &f64, min_y: &f64, cell_size: &f64) -> (Vec<las::Point>, usize, usize) {
    let mut cell = Vec::new();
    
    for wrapped_point in (&mut las_reader).points() {
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
fn multi_grid_division(las_file: &mut las::Reader, cell_size: f64) -> Vec<Vec<Vec<las::Point>>>{
    let mut threads = vec![];

    let min_x = las_file.header().bounds().min.x;
    let min_y = las_file.header().bounds().min.y;
    let max_x = las_file.header().bounds().max.x;
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
            let mut file = las_file;
            threads.push(thread::spawn(move || /*-> (Vec<las::Point>, usize, usize)*/ {
                thread_grid(&mut file, i, j, &min_x, &min_y, &cell_size)
            }));
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
//---------------------------------------Histograms-------------------------------------------

//Function to get histogram of points hegihts in each cell
fn get_histogram(points: &Vec<las::Point>) -> Vec<(f64, usize)>{
    let min_z = points.iter().map(|p| p.z).fold(f64::NAN, f64::min);
    let max_z = points.iter().map(|p| p.z).fold(f64::NAN, f64::max);
    
    let diff = max_z - min_z;
    let mut histogram = Vec::new();

    for i in (0..diff.ceil() as usize).step_by(2) {
        histogram.push((min_z + i as f64, 0));
    }
    
    for point in points {
        for i in 0..histogram.len() {
            if point.z > histogram[i].0  && point.z <= histogram[i].0 + 2. {
                histogram[i].1 += 1;
                break;
            }
        }
    }
    
    histogram
}

//Gets the histogram of all cells
fn grid_histograms(point_cloud: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<(f64, usize)>>> {
    let mut histograms = Vec::new();
    for i in 0..point_cloud.len(){
        histograms.push(Vec::new());
        for _ in 0..point_cloud[i].len() {
            histograms[i].push(Vec::new());
        }
    }
    for i in 0..point_cloud.len(){
        for j in 0..point_cloud[i].len(){
            histograms[i][j] = get_histogram(&point_cloud[i][j]);
        }
    }
    histograms
}

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

fn clean_noise(point_cloud: &mut Vec<Vec<Vec<las::Point>>>, up_epsilon: f64, up_limit: usize, down_epsilon:f64, down_limit: usize, diff_threshold: f64) {
    for i in 0..point_cloud.len() {
        for j in 0..point_cloud[i].len() {
            loop {
                if point_cloud[i][j].len() != 0 {
                    let max_z = point_cloud[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::max);
                    let highest_point = point_cloud[i][j].iter().find(|p| p.z == max_z).unwrap();
                    if is_noise(&highest_point, &point_cloud[i][j], up_epsilon, up_limit, diff_threshold) {
                        let index = point_cloud[i][j].iter().position(|p| p == highest_point).unwrap();
                        point_cloud[i][j].remove(index);
                    }
                    else {
                        break;
                    }
                }
                
            }
            loop {
                if point_cloud[i][j].len() != 0 {
                    let min_z = point_cloud[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::min);
                    let lowest_point = point_cloud[i][j].iter().find(|p| p.z == min_z).unwrap();
                    if is_noise(&lowest_point, &point_cloud[i][j], down_epsilon, down_limit, diff_threshold) {
                        let index = point_cloud[i][j].iter().position(|p| p == lowest_point).unwrap();
                        point_cloud[i][j].remove(index);
                    }
                    else {
                        break;
                    }
                }
            }
        }
    }
}

//--------------------------------------------------------------------------------------------

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

//---------------------------------------------------------------------------------------------
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

fn erode_4_8_neighborhood(point_cloud: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>, neighbourhood: usize) -> Vec<Vec<Vec<las::Point>>> {
    let mut neighbourhood8: bool = false;
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<Vec<las::Point>>> = create_padded_point_cloud(&eroded, 1);

    match neighbourhood {
        4 => {
            neighbourhood8 = false;
        },
        8 => {
            neighbourhood8 = true;
        }
        _ => {
            panic!("Invalid neighbourhood size");
        }
    }
    
    for i in 1..padded.len() - 1 {
        for j in 1..padded[i].len() - 1 {
            // At least 1 neighbour in the neighbourhood (top, bottom, right, left)
            if  padded[i][j].len() > 0 && 
                (i == 1 || j == 1 || i == (padded.len() - 2) || j == (padded[i].len() - 2)) && //Border cells
                ( ((padded[i+1][j].len() > 0) as i32) + ((padded[i-1][j].len() > 0) as i32) 
                + ((padded[i][j+1].len() > 0) as i32) + ((padded[i][j-1].len() > 0) as i32) 
                ) >= 1  {
                    eroded[i-1][j-1] = original[i-1][j-1].clone();
            }
            // At least 1 neighbour in the neighbourhood (upright, bottomleft, bottomright, upleft)
            else if padded[i][j].len() > 0 && neighbourhood8  &&
                (i == 1 || j == 1 || i == (padded.len() - 2) || j == (padded[i].len() - 2)) && //Border cells
                ( ((padded[i+1][j+1].len() > 0) as i32) + ((padded[i-1][j-1].len() > 0) as i32) 
                + ((padded[i-1][j+1].len() > 0) as i32) + ((padded[i+1][j-1].len() > 0) as i32) 
                ) >= 1  {
                    eroded[i-1][j-1] = original[i-1][j-1].clone();
            }
            
            // At least 2 neighbours in the neighbourhood (top, bottom, right, left)
            else if padded[i][j].len() > 0 &&   
                ( ((padded[i+1][j].len() > 0) as i32) + ((padded[i-1][j].len() > 0) as i32) 
                + ((padded[i][j+1].len() > 0) as i32) + ((padded[i][j-1].len() > 0) as i32) 
                ) >= 2  { 
                    eroded[i-1][j-1] = original[i-1][j-1].clone();
            
            }
            // At least 2 neighbours in the neighbourhood (upright, bottomleft, bottomright, upleft)
            else if padded[i][j].len() > 0 && neighbourhood8 && 
                ( ((padded[i+1][j+1].len() > 0) as i32) + ((padded[i-1][j-1].len() > 0) as i32) 
                + ((padded[i-1][j+1].len() > 0) as i32) + ((padded[i+1][j-1].len() > 0) as i32) 
                ) >= 2  { 
                    eroded[i-1][j-1] = original[i-1][j-1].clone();
            
            }

            else {
                eroded[i-1][j-1].clear();
	        }
        }
    }
    eroded
}
fn erode_24_neighborhood(point_cloud: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<Vec<las::Point>>> = create_padded_point_cloud(&eroded, 3);

    println!("{:?}, {:?}", padded.len(), padded[padded.len()-1].len());

    for i in 3..padded.len() - 3 { //3..[32]
        for j in 3..padded[i].len() - 3 { //3..[32]
            let mut has_neighbour = false;
            if padded[i][j].len() > 0 {
                for k in j-3..j+4 { //29-35
                    if ((padded[i-3][k].len() > 0) as i32 
                        + (padded[i+3][k].len() > 0) as i32 
                        ) >= 1 {
                            has_neighbour = true;
                            break;
                    }
                }
                if !has_neighbour {
                    for k in i-3..i+4 {
                        if ((padded[k][j-3].len() > 0) as i32 
                            + (padded[k][j+3].len() > 0) as i32 
                            ) >= 1 {
                                has_neighbour = true;
                                break;
                        }
                    }
                    if !has_neighbour {
                        eroded[i-3][j-3].clear();
                    }
                }  
            }
        }
    }
    eroded
}

// CVR: erode
fn erode(filtered: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>, neighbourhood: i32) -> Vec<Vec<Vec<las::Point>>> {

    match neighbourhood {
        4 => {
            return erode_4_8_neighborhood(filtered, original, 4);
        },
        8 => {
            return erode_4_8_neighborhood(filtered, original, 8);
        }, 
        24 => {
            return erode_24_neighborhood(filtered, original);
        },
        _ => {
            panic!("Invalid neighbourhood size");
        }
    }
}

fn dilate2(filtered: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
    let mut dilated = filtered.clone();
    let padded: Vec<Vec<Vec<las::Point>>> = create_padded_point_cloud(&dilated, 1);
    
    for i in 1..padded.len() - 1{
        for j in 1..padded[i].len() - 1 {
            if  // At least 2 neighbours
                ( ((padded[i+1][j].len() > 0) as i32) + ((padded[i-1][j].len() > 0) as i32) 
                   + ((padded[i][j+1].len() > 0) as i32) + ((padded[i][j-1].len() > 0) as i32) 
                ) >= 2  {
                dilated [i-1][j-1] = original[i-1][j-1].clone();
            } else {
                dilated [i-1][j-1].clear();
	        }
        }
    }
    dilated
}
//-----------------------------------------------------------------------------------------
//---------------------------------------WRITING-------------------------------------------
//Writes into file a single cell
fn write_cell_las(point_cell: &Vec<las::Point>, raw_header: raw::Header, output: &String) {   
    let mut writer = Writer::from_path(output, las::Header::from_raw(raw_header).unwrap()).unwrap();

    for point in point_cell{
        let mut point = point.clone();
        if point.return_number > 5 {
            point.return_number = 5;
        }
        writer.write(point.clone()).unwrap();
    }
    writer.close().unwrap();
}

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
                    writer.write(point.clone()).unwrap();
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
    let gridded = grid_division(&mut reader, 16.7);
    println!("Grid division time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Filtering
    let now = time::Instant::now();
    let filtered = filter_cells(&gridded, 10, 2, 5, 20);
    println!("Filtering time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Erosions
    let now = time::Instant::now();
    let eroded4 = erode(&filtered, &gridded, 4);
    let eroded8 = erode(&eroded4, &gridded, 8);
    let eroded24 = erode(&eroded8, &gridded, 24);
    println!("Erosion time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Dilation
    let now = time::Instant::now();
    let dilated = dilate2(&eroded24, &gridded);
    println!("Dilation time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Writing las/laz and csv
    let now = time::Instant::now();
    write_las(&dilated, raw_header, output);
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

/*  POR HACER */
//1. Medir tiempos
//2. Necesario hacer concurrentes las funciones de limpieza de ruido y filtrado
//3. Como detecto ahora las rectas? (RANSAC?)