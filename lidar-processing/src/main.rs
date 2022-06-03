use las::{Read, Reader, Writer, Write, point::Classification, raw};
use std::fs::File;
use std::env;

// Divide the point cloud into grid of size MxM cells
fn grid_division(las_file: &mut las::Reader, grid_size:usize) -> Vec<Vec<Vec<las::Point>>>{
    let min_x = las_file.header().bounds().min.x;
    let min_y = las_file.header().bounds().min.y;
    let max_x = las_file.header().bounds().max.x;
    let max_y = las_file.header().bounds().max.y;
    let divisions_x = (max_x - min_x) / grid_size as f64;
    let divisions_y = (max_y - min_y) / grid_size as f64;

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

//FALTAN ANALIZAR BORDES
fn reconstruct(filtered: & Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
    let mut reconstructed = filtered.clone();
    
    for i in 1..filtered.len() - 1{
        for j in 1..filtered[i].len() - 1 {
            if filtered[i][j].len() == 0 && (filtered[i+1][j].len() > 0
                                            || filtered[i-1][j].len() > 0
                                            || filtered[i][j+1].len() > 0
                                            || filtered[i][j-1].len() > 0) {
                reconstructed[i][j] = original[i][j].clone();
            }
        }
    }
    reconstructed
}

// CVR: erode
fn erode(filtered: & Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
    let mut eroded = filtered.clone();
    
    for i in 1..filtered.len() - 1{
        for j in 1..filtered[i].len() - 1 {
            if  filtered[i][j].len() > 0 &&  
		// At least 2 neighbours
                ( ((filtered[i+1][j].len() > 0) as i32) + ((filtered[i-1][j].len() > 0) as i32) 
                   + ((filtered[i][j+1].len() > 0) as i32) + ((filtered[i][j-1].len() > 0) as i32) 
                ) >= 2  {
                eroded[i][j] = original[i][j].clone();
            } else {
                eroded[i][j].clear();
	    }
        }
    }
    eroded
}

// CVR: dilate
fn dilate(filtered: & Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
    let mut dilated = filtered.clone();
    
    for i in 1..filtered.len() - 1{
        for j in 1..filtered[i].len() - 1 {
            if  // At least 2 neighbours
                ( ((filtered[i+1][j].len() > 0) as i32) + ((filtered[i-1][j].len() > 0) as i32) 
                   + ((filtered[i][j+1].len() > 0) as i32) + ((filtered[i][j-1].len() > 0) as i32) 
                ) >= 2  {
                dilated [i][j] = original[i][j].clone();
            } else {
                if filtered[i][j].len() > 0 {
                    dilated [i][j].clear();
                }
	    }
        }
    }
    dilated
}

/*fn calculate_grid(header: raw::Header) -> usize {

}*/

fn remove_ground(point_cloud: &mut Vec<Vec<Vec<las::Point>>>) {

    for i in 0..point_cloud.len() {
        for j in 0..point_cloud[i].len() {
            point_cloud[i][j].retain(|p| p.classification !=  Classification::Ground);
        }
    }
}

//---------------------------------------WRITING-------------------------------------------
//Writes into file a single cell
fn cell2las(point_cell: &Vec<las::Point>, raw_header: raw::Header, output: &String) {   
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
fn grid2las(point_cloud: &Vec<Vec<Vec<las::Point>>>, raw_header: raw::Header, output: &String) {
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
//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------
fn main() {
    let args: Vec<String> = env::args().collect();
    let input = &args[1];
    let output = &args[2];

    let mut reader = match Reader::from_path(input) {
        Ok(reader) => reader,
        Err(err) => {
            println!("{:?}", err);
            return;
        }
    };

    let mut file = File::open(input).unwrap();
    let raw_header = raw::Header::read_from(&mut file).unwrap(); //Leemos el header original para quedarnos con los datos de version, padding, scales...

    let mut gridded = grid_division(&mut reader, 60);
    // CVR // clean_noise(&mut gridded, 12., 2, 5., 15, 100.);
    remove_ground(&mut gridded);
    let mut filtered = filter_cells(&mut gridded, 10, 2, 5, 20);
    // CVR // let reconstructed = reconstruct(&mut filtered, &gridded);
    // CVR // grid2las(&reconstructed, raw_header, output);
    let mut eroded = erode(&mut filtered, &gridded);
    let mut dilated = dilate(&mut eroded, &gridded);
    grid2las(&mut dilated, raw_header, output);
}

/*  POR HACER */
// Filtrar vegetación aislada antes de reconstruir
//1. Medir tiempos
//2. Necesario hacer concurrentes las funciones de limpieza de ruido y filtrado
//3. Como detecto ahora las rectas? (RANSAC?)