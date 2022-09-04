use las::Point;

//-----------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------

pub fn convert_to_binary(point_cloud: &Vec<Vec<Vec<Point>>>) -> Vec<Vec<bool>> {
    let mut binary = Vec::new();
    for i in 0..point_cloud.len() {
        binary.push(Vec::new());
        for _ in 0..point_cloud[i].len() {
            binary[i].push(false);
        }
    }
    
    for i in 0..binary.len() {
        for j in 0..binary[i].len() {
            if point_cloud[i][j].len() > 0 {
                binary[i][j] = true;
            }
            else {
                binary[i][j] = false;
            }
        }
    }
    binary
}

//-----------------------------------------------PADDING MATRIX----------------------------------------------
fn empty_vector(num: usize) -> Vec<bool> {
    let mut vec: Vec<bool> = Vec::new();
    for _ in 0..num {
        vec.push(false);
    }
    vec
}

fn create_padded_point_cloud(point_cloud: &Vec<Vec<bool>>, padding: usize) -> Vec<Vec<bool>> {
    let mut padded_point_cloud: Vec<Vec<bool>> = Vec::new();
    let empty_cells: Vec<bool> = empty_vector(padding);

    for i in 0..padding { //Add padding rows to the top
        padded_point_cloud.push(Vec::new());
        for _ in 0..(point_cloud[i].len() + padding*2) {
            padded_point_cloud[i].push(false);
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
            padded_point_cloud[point_cloud.len()+i].push(false);
        }
    }
    padded_point_cloud
}

//-----------------------------------------------EROSIONS----------------------------------------------
fn erode_4(point_cloud: &Vec<Vec<bool>>) -> Vec<Vec<bool>> {
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<bool>> = create_padded_point_cloud(&eroded, 1);
    
    for i in 1..padded.len() - 1 {
        for j in 1..padded[i].len() - 1 {
            // At least 1 neighbour in the neighbourhood (top, bottom, right, left)
            if  padded[i][j] == true {
                let min_neighbours = 
                    if i == 1 || j == 1 || i == (padded.len() - 2) || j == (padded[i].len() - 2) { //Border cells
                        1
                    }
                    else {
                        2
                    };

                if ( ((padded[i+1][j] == true) as i32) + ((padded[i-1][j] == true) as i32) 
                    + ((padded[i][j+1] == true) as i32) + ((padded[i][j-1] == true) as i32) 
                    ) >= min_neighbours  {
                        eroded[i-1][j-1] = true;
                }

                else {
                    eroded[i-1][j-1] = false;
                }
            }
        }
    }
    eroded
}

fn erode_neighborhood(point_cloud: &Vec<Vec<bool>>, padding: usize) -> Vec<Vec<bool>> {
    let mut eroded = point_cloud.clone();
    let padded: Vec<Vec<bool>> = create_padded_point_cloud(&eroded, padding);

    for i in padding..padded.len() - padding {
        for j in padding..padded[i].len() - padding {
            let mut neighbours_per_level = Vec::new();
            for _ in 0..padding {
                neighbours_per_level.push(0);
            }
            
            let mut has_neighbour = false;
            
            if padded[i][j] == true {
                let border_cell = 
                if i == padding || j == padding || i == (padded.len() - (padding + 1) ) || j == (padded[i].len() - (padding + 1) ) { //Border cells
                    true
                }
                else {
                    false
                };

                for k in i-padding..i {
                    for l in j-padding..j+padding+1 {
                        if padded[k][l] == true  {
                            neighbours_per_level[k-(i-padding)] += 1;
                        }
                        if padded[i+padding-(i-k)][l] == true {
                            neighbours_per_level[k-(i-padding)] += 1;
                        }
                    }
                }

                for x in 0..neighbours_per_level.len() {
                    if border_cell && neighbours_per_level[x] >= 1 {
                        has_neighbour = true;
                    }
                    else if !border_cell && i-x >= padding && neighbours_per_level[x] >= 1 {
                        has_neighbour = true;
                    }
                    else if !border_cell && neighbours_per_level[x] >= 2 {
                        has_neighbour = true;
                    }
                    else {
                        has_neighbour = false;
                        break;
                    }
                }

                
                
                if !has_neighbour {
                    for l in j-padding..j {
                        if padded[i][l] == true  {
                            neighbours_per_level[l-(j-padding)] += 1;
                        }
                        if padded[i][j+padding-(j-l)] == true  {
                            neighbours_per_level[l-(j-padding)] += 1;
                        }
                    }
                    
                    for x in 0..neighbours_per_level.len() {
                        if border_cell && neighbours_per_level[x] >= 1 {
                            has_neighbour = true;
                        }
                        else if !border_cell && i-x >= padding && neighbours_per_level[x] >= 1 {
                            has_neighbour = true;
                        }
                        else if !border_cell && neighbours_per_level[x] >= 2 {
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
                        eroded[i-padding][j-padding] = true
                    },
                    false => {
                        eroded[i-padding][j-padding] = false ;
                    }
                }
            }
        }
    }
    eroded
}

pub fn erode(filtered: &Vec<Vec<bool>>, neighbourhood: usize) -> Vec<Vec<bool>> {

    match neighbourhood {
        0 => {
            return erode_4(filtered);
        },
        _ => {
            return erode_neighborhood(filtered, neighbourhood);
        }
    }
}

//-----------------------------------------------DILATION----------------------------------------------
pub fn dilate(filtered: &Vec<Vec<bool>>) -> Vec<Vec<bool>> {
    let mut dilated = filtered.clone();
    let padded: Vec<Vec<bool>> = create_padded_point_cloud(&dilated, 1);
    
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
                if ((padded[i-1][k] == true ) as i32 
                    + (padded[i+1][k] == true ) as i32 
                ) >= min_neighbours {
                    has_neighbour = true;
                    break;
                }
            }
            
            if !has_neighbour {
                if ((padded[i][j-1] == true ) as i32 
                    + (padded[i][j+1] == true ) as i32 
                ) >= min_neighbours {
                    has_neighbour = true;
                }
            }  

            match has_neighbour {
                true => {
                    dilated[i-1][j-1] = true;
                },
                false => {
                    dilated[i-1][j-1] = false;
                }
            }
        }
    }
    dilated
}