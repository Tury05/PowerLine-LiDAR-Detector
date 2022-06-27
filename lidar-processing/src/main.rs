use las::{Read, Reader, Writer, Write, point::Classification, raw, Header};
use std::{fs::File, fs::metadata, fs::create_dir, fs::read_dir, env, time, path::Path, ffi::OsStr};
use rand::{Rng, prelude::*};
use std::thread;

/*fn multi_grid(points_vec: &Vec<las::Point>, pos_x: usize, pos_y: usize, min_x: &f64, min_y: &f64, cell_size: &f64) -> (Vec<las::Point>, usize, usize) {
    let mut cell = Vec::new();
    
    for point in points_vec {
        if point.x >= (cell_size*pos_x as f64) + min_x && point.x < (cell_size*(pos_x+1) as f64) + min_x{
            if point.y >= (cell_size*pos_y as f64) + min_y && point.y < (cell_size*(pos_y+1) as f64) + min_y {
                cell.push(point.clone());
            }
        }       
    }
    (cell, pos_x, pos_y)
}

//Divide the point cloud into grid of size MxM cells
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

    let points = las_file.points().map(|r| r.unwrap()).collect::<Vec<_>>();

    for i in 0..grid_size {
        for j in 0..grid_size {
            let points_vec = points.clone();
            threads.push(thread::spawn(move || 
                multi_grid(&points_vec, i, j, &min_x, &min_y, &cell_size)
            ));
        }
    }
    for thread in threads {
        let (cell, pos_x, pos_y) = thread.join().unwrap();
        grids[pos_x][pos_y] = cell;
    }
    grids
}*/

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

//-----------------------------------------------------------------------------------
/*//Euclidean distance between two points
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

fn clean_noise(point_cloud: &mut Vec<Vec<Vec<las::Point>>>, up_epsilon: f64, up_limit: usize, diff_threshold: f64) {
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
                else {
                    break;
                }
            }
        }
    }
}*/

//-------------------------------------------FILTER HEIGHT DENSITYS-------------------------------------------------

fn mean_zvalue(points: &Vec<las::Point>) -> f64 {
    let mut sum = 0.0;
    for point in points {
        sum += point.z;
    }
    sum / points.len() as f64
}

fn filter_covariance(point_cloud: &Vec<Vec<Vec<las::Point>>>, epsilon: f64) -> Vec<Vec<Vec<las::Point>>> {
    let mut filtered_point_cloud = point_cloud.clone();
    for i in 0..point_cloud.len() {
        for j in 0..point_cloud[i].len() {
            if point_cloud[i][j].len() != 0 {
                let mean_z = mean_zvalue(&point_cloud[i][j]);
                let mut count = 0;
                let mut sum = 0.0;
                for point in &point_cloud[i][j] {
                        sum += (point.z - mean_z).powi(2);
                        count += 1;
                }
                let covariance = (sum / count as f64).sqrt();
                if covariance > epsilon {
                    filtered_point_cloud[i][j].clear();
                }
            }
        }
    }
    filtered_point_cloud
}

fn filter_height_density(point_cell: &Vec<las::Point>, min_density: usize, empty:usize , non_empty_cont:usize, non_empty: usize , min_iter: usize, end: usize) -> Vec<las::Point> {
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
fn filter_cells(point_cloud: &Vec<Vec<Vec<las::Point>>>, min_density: usize, empty:usize , non_empty_cont: usize, non_empty:usize, min_iter: usize, end: usize) -> Vec<Vec<Vec<las::Point>>> {   
    let mut filtered_point_cloud = Vec::new();
    for i in 0..point_cloud.len(){
        filtered_point_cloud.push(Vec::new());
        for j in 0..point_cloud[i].len(){
            filtered_point_cloud[i].push(filter_height_density(&point_cloud[i][j], min_density, empty, non_empty_cont, non_empty, min_iter, end));
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

fn continuity(point_cloud: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>, erode: bool) -> Vec<Vec<Vec<las::Point>>> {
    let padding = 1;
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<Vec<las::Point>>> = create_padded_point_cloud(&eroded, padding);

    for i in 1..padded.len() - 1 {
        for j in 1..padded[i].len() - 1 {
            // At least 1 neighbour in the neighbourhood (top, bottom, right, left)
            
            let min_neighbours = 
                if i == 1 || j == 1 || i == (padded.len() - 2) || j == (padded[i].len() - 2) { //Border cells
                    1
                }
                else {
                    2
                };
                
            if padded[i+1][j].len() > 0 && (((padded[i-1][j].len() > 0) as i32) 
                + ((padded[i-1][j+1].len() > 0) as i32) + ((padded[i-1][j-1].len() > 0) as i32) 
                 >= min_neighbours ) {
                    eroded[i-1][j-1] = original[i-1][j-1].clone();
            }
            else if padded[i-1][j].len() > 0 && (((padded[i+1][j].len() > 0) as i32) 
                + ((padded[i+1][j+1].len() > 0) as i32) + ((padded[i+1][j-1].len() > 0) as i32) 
                 >= min_neighbours ) {
                    eroded[i-1][j-1] = original[i-1][j-1].clone();
            }
            else if (padded[i-1][j-1].len() > 0) && ((padded[i+1][j].len() > 0) as i32 
                    + (padded[i+1][j+1].len() > 0) as i32 
                    + (padded[i][j+1].len() > 0) as i32 
                    >= min_neighbours){
                eroded[i-1][j-1] = original[i-1][j-1].clone();
            }
            else if (padded[i-1][j+1].len() > 0) && ((padded[i+1][j].len() > 0) as i32 
                    + (padded[i+1][j-1].len() > 0) as i32 
                    + (padded[i][j-1].len() > 0) as i32 
                    >= min_neighbours){
                eroded[i-1][j-1] = original[i-1][j-1].clone();
            }
            else if (padded[i+1][j+1].len() > 0) && ((padded[i-1][j].len() > 0) as i32 
                    + (padded[i-1][j-1].len() > 0) as i32 
                    + (padded[i][j-1].len() > 0) as i32 
                    >= min_neighbours){
                eroded[i-1][j-1] = original[i-1][j-1].clone();
            }
            else if (padded[i+1][j-1].len() > 0) && ((padded[i-1][j].len() > 0) as i32 
                    + (padded[i-1][j+1].len() > 0) as i32 
                    + (padded[i][j-1].len() > 0) as i32 
                    >= min_neighbours){
                eroded[i-1][j-1] = original[i-1][j-1].clone();
            }
            else if erode {
                let empty_vec: Vec<las::Point> = Vec::new();
                eroded[i-1][j-1] = empty_vec;
            }
        }
    }
    eroded
}

//NO FUNCIONA BIEEN
fn erode_continuity(point_cloud: &Vec<Vec<Vec<las::Point>>>, original: &Vec<Vec<Vec<las::Point>>>, neighbourhood: usize, erode: bool) -> Vec<Vec<Vec<las::Point>>> {
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
            let mut count_up: i32 = 0;
            let mut count_down: i32  = 0;
            let mut count_left: i32  = 0;
            let mut count_right: i32  = 0;
            let mut count_adj: i32  = 0;
            let mut has_neighbour = false;
            
        
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
                        count_up += 1;
                        if k == i-1 && l >= j-1 && l <= j+1 {
                            count_adj += 1;
                        }
                    }
                    if padded[i+padding-(i-k)][l].len() > 0 {
                        count_down += 1;
                        if k == i+1 && l >= j-1 && l <= j+1 {
                            count_adj += 1;
                        }
                    }
                }
            }
            if (!border_cell && count_up == count_down && count_up >= 1 && count_adj >= 2) 
                || (border_cell && (count_up == (count_down - padding as i32) || count_up == count_down + padding as i32) && count_up >= 1 && count_adj >= 2) {
                has_neighbour = true;
            }

            if !has_neighbour {
                let mut count_adj = 0;
                for k in i-padding..i+padding+1 {
                    for l in j-padding..j {
                        if padded[k][l].len() > 0 {
                            count_left += 1;
                            if k >= i-1 && k <= i+1 && l == j-1 {
                                count_adj += 1;
                            }
                        }
                        if padded[k][j+padding-(j-l)].len() > 0 {
                            count_right += 1;
                            if k >= i-1 && k <= i+1 && l == j+1 {
                                count_adj += 1;
                            }
                        }
                    }
                }
                if (!border_cell && count_left == count_right && count_left >= 1 && count_adj >= 2)
                    || (border_cell && (count_left == (count_right - padding as i32)|| count_left == (count_right + padding as i32)) && count_left >= 1 && count_adj >= 2) {
                    has_neighbour = true;
                }
            } 

            match has_neighbour {
                true => {
                    eroded[i-padding][j-padding] = 
                        original[i-padding][j-padding].clone();
                },
                false => {
                    if eroded[i-padding][j-padding].len() > 0 && erode {
                        eroded[i-padding][j-padding].clear();
                    }
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

            32 => {
                4
            }
            _ => {
                panic!("Invalid neighbourhood size");
            }
        };

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
                    if (border_cell && neighbours_per_level[x] >= 1) || (!border_cell && neighbours_per_level[x] >= 2) {
                        has_neighbour = true;
                    }
                    else {
                        has_neighbour = false;
                        break;
                    }
                }

                
                
                if !has_neighbour {
                    for k in i-padding..i+padding+1 {
                        for l in j-padding..j {
                            if padded[k][l].len() > 0 {
                                neighbours_per_level[l-(j-padding)] += 1;
                            }
                            if padded[k][j+padding-(j-l)].len() > 0 {
                                neighbours_per_level[l-(j-padding)] += 1;
                            }
                        }
                    }
                    for x in 0..neighbours_per_level.len() {
                        if (border_cell && neighbours_per_level[x] >= 1) || (!border_cell && neighbours_per_level[x] >= 2) {
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
                            points_under_max(&original[i-padding][j-padding], 18.);
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
        8 | 16 | 24 | 32 => {
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
                    dilated[i-1][j-1] = points_under_max(&original[i-1][j-1], 18.);
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
    let filtered = filter_cells(&gridded, 6, 4, 4, 7, 10, 30);
    println!("Filtering time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Erosions
    let eroded4 = erode(&filtered, &gridded, 4);
    let eroded32 = erode(&eroded4, &gridded, 32);
    println!("Erosion time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Dilation
    let now = time::Instant::now();
    let dilated = dilate(&eroded32, &gridded);
    let eroded4 = erode(&dilated, &gridded, 4);
    let eroded16 = erode(&eroded4, &gridded, 16);
    
    //let eroded_cont = continuity(&dilated, &gridded, true);
    //let dilated2 = dilate(&eroded_cont, &gridded);
    println!("Dilation time: {:?} millisecs.", now.elapsed().as_millis());
   
    //Writing las/laz and csv
    let now = time::Instant::now();
    write_las(&eroded16, raw_header, output);
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


//
// Reconectar lineas electricas (buscar vecino mas cercano en una direccion dentro de un radio)