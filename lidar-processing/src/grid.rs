use core::panic;

use las::{Header, Point};

pub fn grid_division(file_header: &Header, point_cloud: Vec<Point>,cell_size: f64) -> Vec<Vec<Vec<Point>>>{
    if cell_size <= 0. {
        panic!("Grid division error. Cell_size must be greater than 0");
    }
    let min_x = file_header.bounds().min.x;
    let min_y = file_header.bounds().min.y;
    let max_x = file_header.bounds().max.x;

    let divisions_x = cell_size;
    let divisions_y = cell_size;

    let grid_size = ((max_x - min_x)/divisions_x).ceil() as usize;
    
    let mut grids:Vec<Vec<Vec<Point>>> = Vec::new();
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

pub fn create_result(cells_list: &Vec<(usize, usize)>, point_cloud: &Vec<Vec<Vec<las::Point>>>, density_thres: usize) -> Vec<Vec<Vec<las::Point>>> {
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
    let mut mean_density = 0;
    let mut count = 0;
    for i in 0..result.len() {
        for j in 0..result[i].len() {
            if result[i][j].len() > 0 {
                let max_z = result[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::max);
                let min_z = result[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::min);
                for x in min_z as i32..max_z as i32 {
                    let density = result[i][j].iter().filter          //Number of points in height range
                    (|p| p.z >= x as f64 && p.z < x as f64 + 1.).count();
                    if density > 0 {
                        mean_density += density;
                        count += 1;
                    }
                }
            }
        }
    }
    if count > 0 {
        mean_density = mean_density/count;
        for i in 0..result.len() {
            for j in 0..result[i].len() {
                let max_z = result[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::max);
                let min_z = result[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::min);
                for w in min_z as i32..max_z as i32 {
                    if result[i][j].iter().filter
                    (|p| p.z >= w as f64 && p.z < w as f64 + 1.).count() > mean_density + density_thres {
                        result[i][j].retain(|x| x.z > w as f64 + 1.);
                    }
                }
            }
        }
    }
    result
}