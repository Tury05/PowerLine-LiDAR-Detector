use las::{Reader, Writer, Write, point::Classification, raw};
use std::fs::File;

pub fn read_las(input: &String) -> (Reader, raw::Header)  {
    let reader = match Reader::from_path(input) {
        Ok(reader) => reader,
        Err(err) => {
            panic!("{:?}", err);
        }
    };

    let mut file = File::open(input).unwrap();
    let raw_header = raw::Header::read_from(&mut file).unwrap(); //Leemos el header original para quedarnos con los datos de version, padding, scales...
    (reader, raw_header)
}

//-----------------------------------------------------------------------------------------
//---------------------------------------WRITING-------------------------------------------

//Writes into file all point cloud
pub fn write_las(point_cloud: &Vec<Vec<Vec<las::Point>>>, raw_header: raw::Header, output: &String) {
    let mut writer = Writer::from_path(output, las::Header::from_raw(raw_header).unwrap()).unwrap();

    for i in 0..point_cloud.len(){
        for j in 0..point_cloud[i].len(){
            if point_cloud[i][j].len() > 0{
                for point in &point_cloud[i][j]{
                    let mut point = point.clone();
                    if point.return_number > 5 {
                        point.return_number = 5;
                    }
                    if point.classification == Classification::TransmissionTower {
                        writer.write(point.clone()).unwrap();
                    }
                }
            }
        }
    }
    writer.close().unwrap();
}

