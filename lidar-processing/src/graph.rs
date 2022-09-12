//------------------------------------BREADTH-FIRST-----------------------------------------
//-----------------------------------------------------------------------------------------

//Checks if cell is visited or it's value is true/false.
fn is_safe(matrix: &Vec<Vec<bool>>, visited: &Vec<Vec<bool>>, i: i32, j: i32) -> bool {
    if i < 0 || j < 0 || i >= matrix.len() as i32 
      || j >= matrix[i as usize].len() as i32
      || visited[i as usize][j as usize] 
      || !matrix[i as usize][j as usize] {
        return false;
    } else { //If not visited and value == true
        return true;
    }
}

//Calculates distance between two cells
fn euclidean_cells(cell1: (usize, usize), cell2: (usize, usize)) -> f64 {
    let (x1, y1) = cell1;
    let (x2, y2) = cell2;
    let x_diff = (x1 as f64 - x2 as f64).powi(2);
    let y_diff = (y1 as f64 - y2 as f64).powi(2);
    (x_diff + y_diff).sqrt()
}

//Search for the most distant cells in connected components
fn most_distant_cells(cells: &Vec<(usize, usize)>) -> ([(usize, usize); 2], f64){
    let mut max_distance = 0.;
    let mut most_distant:[(usize, usize); 2] = [(0, 0), (0, 0)];
    
    for i in 0..cells.len()-1 {
        for j in i+1..cells.len() {
            if euclidean_cells(cells[i], cells[j]) > max_distance {
                max_distance = euclidean_cells(cells[i], cells[j]);
                most_distant = [cells[i], cells[j]];
            }
        }
    }

    (most_distant, max_distance)
}

//Transforms list of cells to maintain to a point cloud.
fn create_result(cells_list: &Vec<(usize, usize)>, point_cloud: &Vec<Vec<Vec<las::Point>>>) -> Vec<Vec<Vec<las::Point>>> {
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
    result
}

//Connected components based filter.
pub fn filter_conn_components(matrix: &Vec<Vec<bool>>, point_cloud: &Vec<Vec<Vec<las::Point>>>, dist_thres:f64, comp_thres: usize) -> Vec<Vec<Vec<las::Point>>> {
    let dx:[i32; 8] = [1, 1, 1, 0, -1, -1, -1, 0]; //Neighbours direction (8-neighbourhood)
    let dy:[i32; 8]  = [1, 0, -1, -1, -1, 0, 1, 1];
    let mut candidates = Vec::new(); 
    
    let mut visited = Vec::new();
    for i in 0..matrix.len() {
        visited.push(Vec::new());
        for _ in 0..matrix[i].len() {
            visited[i].push(false);
        }
    }

    //loop through the entire point cloud (binary matrix)
    for i in 0..matrix.len() {
        for j in 0..matrix[i].len() {
            let mut count = 0;
            let mut cells_group = Vec::new();
            if !visited[i][j] && matrix[i][j] {
                let mut queue = Vec::new();
                queue.push((i, j));
                visited[i][j] = true;
                while queue.len() != 0 { //Visit neighbours in every direction
                    let (x, y) = queue.pop().unwrap();
                    cells_group.push((x, y));
                    count += 1;
                    for k in 0..8 {
                        let (x1, y1) = (x as i32 + dx[k], y as i32 + dy[k]);
                        if is_safe(matrix, &visited, x1, y1) {
                            queue.push((x1 as usize, y1 as usize));
                            visited[x1 as usize][y1 as usize] = true;
                        }
                    }
                }
                let (_, distance) = most_distant_cells(&cells_group);

                if count > comp_thres && distance > dist_thres {
                    candidates.append(&mut cells_group);
                }
            }
        }
    }
    create_result(&candidates, point_cloud)
}