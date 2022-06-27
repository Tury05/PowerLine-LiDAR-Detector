use las::{Read, Reader, raw};
use std::{fs::File, fs::metadata, fs::read_dir, env, time, ffi::OsStr};

fn create_matrix(filtered: &mut las::Reader, ground_truth: &mut las::Reader, original: &raw::Header) -> Vec<u32> {
    let filtered_vec = filtered.points().map(|r| r.unwrap()).collect::<Vec<_>>();
    let ground_truth_vec = ground_truth.points().map(|r| r.unwrap()).collect::<Vec<_>>();
    let mut matrix = Vec::new(); //TN, FN, TP, FP
    for _ in 0..6 {
        matrix.push(0);
    }

    for point in &filtered_vec{
        let mut found = false;
        for ground_truth_point in &ground_truth_vec{
            if *point == *ground_truth_point {
                found = true;
            }
        }
        if found{
            matrix[2] += 1;
        }
        else{
            matrix[3] += 1;
        }
    }

    for point in &ground_truth_vec{
        let mut found = false;
        for filtered_point in &filtered_vec{
            if *point == *filtered_point {
                found = true;
            }
        }
        if !found{
            matrix[1] += 1;
        }
    }
    matrix[0] = original.number_of_point_records - matrix[1] - matrix[2] - matrix[3];
    matrix
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let path1 = &args[1];
    let path2 = &args[2];
    let path3 = &args[3];

    let mut vec1 = Vec::new();
    let mut vec2 = Vec::new();
    let mut vec3 = Vec::new();

    let mut count = 0;
    let mut main_matrix = Vec::new();
    for _ in 0..7 {
        main_matrix.push(0);
    }

    let md_filtered = metadata(path1).unwrap();
    let md_gt = metadata(path2).unwrap();
    let md_original = metadata(path3).unwrap();

    if md_filtered.is_dir() {
        if !md_gt.is_dir() {
            panic!("{} is not a directory", path2);
        }
        if !md_original.is_dir() {
            panic!("{} is not a directory", path3);
        }
        let paths_filtered = read_dir(path1).unwrap();
        for path in paths_filtered {
            vec1.push(path.unwrap());
        }
        let paths_gt = read_dir(path2).unwrap();
        for path in paths_gt {
            vec2.push(path.unwrap());
        }
        let paths_original = read_dir(path3).unwrap();
        for path in paths_original {
            vec3.push(path.unwrap());
        }

        for path in vec1 {
            let path_filtered = path.path();
            let extension = path_filtered
                .extension()
                .and_then(OsStr::to_str);
            if extension == Some("las") || extension == Some("laz") {
                for path_gt in &vec2 {
                    let path_gt = path_gt.path();
                    for path_original in &vec3 {
                        let path_original = path_original.path();
                        if path_filtered.file_name().unwrap().to_str().unwrap()[9..] == path_gt.file_name().unwrap().to_str().unwrap()[13..] 
                        && path_filtered.file_name().unwrap().to_str().unwrap()[11..] == *path_original.file_name().unwrap().to_str().unwrap() {
                            let filename_filtered = path_filtered.file_name().unwrap().to_str().unwrap();
                            let filtered_path = format!("{}/{}", path1, filename_filtered);
                            let filename_gt = path_gt.file_name().unwrap().to_str().unwrap();
                            let gt_path = format!("{}/{}", path2, filename_gt);
                            let filename_original = path_original.file_name().unwrap().to_str().unwrap();
                            let original_path = format!("{}/{}", path3, filename_original);

                            let mut reader_filtered = Reader::from_path(filtered_path.clone()).unwrap();
                            let reader_gt = Reader::from_path(gt_path.clone());
                            let mut reader_gt = match reader_gt {
                                Ok(reader_gt2) => reader_gt2,
                                Err(e) => continue
                            };

                            let mut file = File::open(original_path).unwrap();
                            let raw_header_or = raw::Header::read_from(&mut file).unwrap();
                            let points_or = raw_header_or.number_of_point_records;

                            let mut file = File::open(gt_path).unwrap();
                            let raw_header_gt = raw::Header::read_from(&mut file).unwrap();
                            let points_gt = raw_header_gt.number_of_point_records;

                            let mut file = File::open(filtered_path).unwrap();
                            let raw_header_fil = raw::Header::read_from(&mut file).unwrap();
                            let points_fil = raw_header_fil.number_of_point_records;

                            

                            count += 1;
                            let cell_confusion_matrix = create_matrix(&mut reader_filtered, &mut reader_gt, &raw_header_or);
                            println!("El fichero {} tiene: {:?} TN; {:?} FN; {:?} TP; {:?} FP. || GT_POINTS: {}, FILTERED_POINTS: {} ORIGINAL_POINTS: {}", 
                                filename_original, cell_confusion_matrix[0], cell_confusion_matrix[1], cell_confusion_matrix[2], cell_confusion_matrix[3], points_gt, points_fil, points_or);
                            for i in 0..4 {
                                main_matrix[i] += cell_confusion_matrix[i];
                            }
                            main_matrix[4] += points_gt;
                            main_matrix[5] += points_fil;
                            main_matrix[6] += points_or;
                        }
                    }
                }
            }
        }
        println!("Resultados totales de {} tiles: {:?} TN; {:?} FN; {:?} TP; {:?} FP, {:?} POINTS GT, {:?} POINTS FILTERED", 
            count, main_matrix[0], main_matrix[1], main_matrix[2], main_matrix[3], main_matrix[4], main_matrix[5]);
    
    }
}
/*fn create_matrix(filtered_string: &String, ground_truth_string: &String) -> Vec<usize> {
    let mut f_neg = 0;
    let mut f_pos = 0;
    let mut t_pos = 0;
    let mut matrix =Vec::new(); //FP, FN, VP

    let path_filtered = filtered_string.clone();
    let filtered_thread = thread::spawn(move || {
        let mut filtered = Reader::from_path(path_filtered).unwrap();
        let mut filtered_vec = Vec::new();
        for wrapped_point in filtered.points() {
            let point = wrapped_point.unwrap();
            filtered_vec.push(point.clone());
        }
        filtered_vec
    });
    
    let path_gt= ground_truth_string.clone();
    let gt_thread = 
        thread::spawn(move || {
            let mut ground_truth = Reader::from_path(path_gt).unwrap();
            let mut ground_truth_vec = Vec::new();
            for wrapped_point in ground_truth.points() {
                let point = wrapped_point.unwrap();
                ground_truth_vec.push(point.clone());
            }
            ground_truth_vec
        });

    let mut filtered_vec = filtered_thread.join().unwrap();
    let ground_truth_vec = gt_thread.join().unwrap();

    let f_vec = filtered_vec.clone();
    let mut gt_vec = ground_truth_vec.clone();
    
    for point in f_vec {
        let mut found = false;
        for ground_truth_point in gt_vec.clone() {
            if point == ground_truth_point {
                found = true;
                let index = gt_vec.iter().position(|x| *x == ground_truth_point).unwrap();
                gt_vec.remove(index);
                break;
            }
        }
        if found{
            t_pos += 1;
        }
        else{
            f_pos += 1;
        }
    }

    for point in ground_truth_vec.clone(){
        let mut found = false;
        for filtered_point in filtered_vec.clone(){
            if point == filtered_point {
                found = true;
                let index = filtered_vec.iter().position(|x| *x == filtered_point).unwrap();
                filtered_vec.remove(index);
                break;
            }
        }
        if !found{
            f_neg += 1;
        }
    }
    matrix.push(f_pos);
    matrix.push(f_neg);
    matrix.push(t_pos);    

    matrix
} */