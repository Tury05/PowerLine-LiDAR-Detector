use las::{Read, Reader, Writer, Header, Write};
use std::fs::File;

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
                    if point.y >= (divisions_y*i as f64) + min_y && point.y < (divisions_y*(i+1) as f64) + min_y {
                        grids[i][j].push(point);
                        continue 'outer;
                    }
                }
            } 
        }
    }
    grids
}

fn write_las(point_cloud: &Vec<las::Point>) {

    let mut header = Header::default();
    for point in point_cloud{
        header.add_point(point);
    }

    let file = File::create("/home/tury/Escritorio/output.las").unwrap();
    let mut writer = Writer::new(file, Default::default()).unwrap();

    for point in point_cloud{
        println!("{:?}",point);
        writer.write(point.clone()).unwrap();
    }
    println!("done");

}

fn main() {
    let mut reader = Reader::from_path("../data/input.las").unwrap();
    let gridded = grid_division(&mut reader, 50);
    write_las(&gridded[0][0]);   

}