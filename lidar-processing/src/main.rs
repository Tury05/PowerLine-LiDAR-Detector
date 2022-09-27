use las::Read;
use std::{fs::metadata, fs::create_dir, fs::read_dir, env, time, path::Path, ffi::OsStr};

mod io; 
mod histogram;
mod morph;
mod graph;
mod ground;
mod grid;

//----------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------

fn execute_algorithms(input: &String, output: &String) {
    let mut readed_items = io::read_las(input); 

    println!("----------Processing file: {}----------", input);

    //Ground Reduction
    let now = time::Instant::now();
    let ground_reduced = ground::ground_reduction(&mut readed_items.0, 0.01);
    println!("Ground Reduction time: {:?} millisecs.", now.elapsed().as_millis());

    //Grid Division
    let now = time::Instant::now();
    let gridded = grid::grid_division(readed_items.0.header(), ground_reduced, 7.5);
    println!("Grid division time: {:?} millisecs.", now.elapsed().as_millis());

    //Histogram based filtering
    let now = time::Instant::now();
    let filtered = histogram::histogram_filter(&gridded, 6, 4, 4, 7, 30);
    println!("Local Histogram Filtering time: {:?} millisecs.", now.elapsed().as_millis());

    let binary = morph::convert_to_binary(&filtered);
   
    //Morphological operations
    let now = time::Instant::now();
    
    let eroded4 = morph::erode(&binary,0);
    let eroded32 = morph::erode(&eroded4,5);
    
    let dilated = morph::dilate(&eroded32);
    
    let eroded4 = morph::erode(&dilated, 0);
    let eroded32 = morph::erode(&eroded4, 4);
    
    println!("Morphological operations time: {:?} millisecs.", now.elapsed().as_millis());

    //Graph based filtering
    let now = time::Instant::now();
    let conn_comps = graph::filter_conn_components(&eroded32, 18., 30);
    println!("Connected Components Filtering time: {:?} millisecs.", now.elapsed().as_millis());

    //Density voxel filtering
    let now = time::Instant::now();
    let result = grid::create_result(&conn_comps, &gridded, 45);
    println!("Density voxel Filtering time: {:?} millisecs.", now.elapsed().as_millis());
    
    //Writing output file
    let now = time::Instant::now();
    io::write_las(&result, readed_items.1, output);
    println!("Writing time: {:?} millisecs.", now.elapsed().as_millis());
}

fn main() {
    let total_time = time::Instant::now();
    let args: Vec<String> = env::args().collect();

    if args.len() != 3 {
        panic!("Error: Program must be executes with command \"./lidar-processing <input> <output>\"");
    }

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
    println!("Total time for {:?} cells: {:?} milisecs.", num_cells, total_time.elapsed().as_millis());
}