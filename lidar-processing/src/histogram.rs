use las::Point;

//---------------------------------------Histograms-------------------------------------------

//Function to get histogram of points hegihts in each cell
#[allow(dead_code)]
pub fn get_histogram(points: &Vec<Point>) -> Vec<(f64, usize)>{
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

//-------------------------------------------FILTER HEIGHT DENSITYS-------------------------------------------------

//Local Histogram based filter
fn process_cell(point_cell: &Vec<Point>, min_density: usize, empty:usize , non_empty_cont:usize, non_empty: usize, end: usize) -> Vec<Point> {
    let mut cell = Vec::new();
    let mut candidate = false;
    let mut count_non_empty = 0;
    let mut count_empty = 0;
    let mut total_count_non_empty = 0;
    
    let max_z = point_cell.iter().map(|p| p.z).fold(f64::NAN, f64::max);
    let min_z = point_cell.iter().map(|p| p.z).fold(f64::NAN, f64::min);
    for i in 0..end {
        if (max_z - (i as f64)) > min_z{
            let density = point_cell.iter().filter          //Number of points in height range
                    (|p| p.z <= max_z - (i as f64) && p.z > max_z - (i as f64 + 1.)).count();
            
            if density > min_density {      //Height range is not empty
                count_non_empty += 1;
                total_count_non_empty += 1;
                count_empty = 0;
            
            } else {    //Height range is empty
                count_empty += 1;
                count_non_empty = 0;
            }

            if count_non_empty >= non_empty_cont || total_count_non_empty >= non_empty {    //Multiple non empty height range (cell is not candidate)
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
pub fn histogram_filter(point_cloud: &Vec<Vec<Vec<Point>>>, min_density: usize, empty:usize , non_empty_cont: usize, non_empty:usize, end: usize) -> Vec<Vec<Vec<Point>>> {   
    let mut filtered_point_cloud = Vec::new();
    for i in 0..point_cloud.len(){
        filtered_point_cloud.push(Vec::new());
        for j in 0..point_cloud[i].len(){
            filtered_point_cloud[i].push(process_cell(&point_cloud[i][j], min_density, empty, non_empty_cont, non_empty, end));
        }
    }
    filtered_point_cloud
}