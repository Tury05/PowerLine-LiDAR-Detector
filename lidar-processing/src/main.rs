use las::{Read, Reader, Writer, Write, Builder, point::Format, Transform, raw::Header};
use std::fs::File;
use std::env;

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

fn points_above_height(points: &Vec<Vec<Vec<las::Point>>>, height: f64) -> Vec<Vec<Vec<las::Point>>> {
    let mut point_cloud = points.clone();
    for i in 0..point_cloud.len(){
        for j in 0..point_cloud[i].len(){
            let min_z = point_cloud[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::min);
            point_cloud[i][j].retain(|point| point.z > min_z+height);
        }
    }
    point_cloud
}

fn filter_by_density_and_height(points: &Vec<Vec<Vec<las::Point>>>, height: f64) -> Vec<Vec<Vec<las::Point>>> {
    let mut point_cloud = points.clone();
    let total_points_num = point_cloud.len() * point_cloud[0].len() * point_cloud[0][0].len();
    let grid_area = (point_cloud.len() * point_cloud[0].len()) as f64;
    let density = total_points_num as f64 / grid_area;
    let threshold = (2.*density).sqrt();
    for i in 0..point_cloud.len(){
        for j in 0..point_cloud[i].len(){
            let min_z = point_cloud[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::min);
            let max_z = point_cloud[i][j].iter().map(|p| p.z).fold(f64::NAN, f64::max);
            let count = point_cloud[i][j].len();
            
            if (count as f64) < threshold {
                continue;
            }

            else if max_z - min_z > height {
                point_cloud[i][j].clear();
            }
        }
    }
    point_cloud
}

fn cell2las(point_cloud: &Vec<las::Point>, point_format: u8) {

    let mut builder = Builder::default();
    let transform = Transform { scale: 0.01, offset: 0. };
    let scales = las::Vector{x:transform, y:transform, z:transform};
    builder.transforms = scales;
    builder.point_format = Format::new(point_format).unwrap();

    let file = File::create("/home/tury/Escritorio/output.las").unwrap();
    let mut writer = Writer::new(file, builder.into_header().unwrap()).unwrap();

    for point in point_cloud{
        writer.write(point.clone()).unwrap();
    }
    writer.close().unwrap();

}

fn grid2las(point_cloud: &Vec<Vec<Vec<las::Point>>>, point_format: u8, output_path: &str) {

    let mut raw_header = Header::default();
    raw_header.x_scale_factor = 0.001;
    raw_header.y_scale_factor = 0.001;
    raw_header.z_scale_factor = 0.001;    

    let mut builder = Builder::new(raw_header).unwrap();
    let transform = Transform { scale: 10., offset: 0. };
    let scales = las::Vector{x:transform, y:transform, z:transform};
    builder.transforms = scales;
    builder.point_format = Format::new(point_format).unwrap();

   let mut writer = Writer::from_path(output_path, builder.into_header().unwrap()).unwrap();

    for i in 0..point_cloud.len(){
        for j in 0..point_cloud[i].len(){
            if point_cloud[i][j].len() > 0{
                for point in &point_cloud[i][j]{
                    let mut point = point.clone();
                    if point.return_number > 5 {
                        point.return_number = 5;
                    }
                    point.x *= 1000.;
                    point.y *= 1000.;
                    point.z *= 1000.;
                    writer.write(point.clone()).unwrap();
                }
            }
        }
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();
    
    let mut reader = match Reader::from_path(&args[1]) {
        Ok(reader) => reader,
        Err(err) => {
            println!("{:?}", err);
            return;
        }
    };
    
    let header = reader.header();
    let format = header.point_format().to_u8().unwrap();
    let gridded = grid_division(&mut reader, 60);
    let above = points_above_height(&gridded, 13.);
    let filtered = filter_by_density_and_height(&above, 16.);
    grid2las(&filtered, format, &args[2]);   

}