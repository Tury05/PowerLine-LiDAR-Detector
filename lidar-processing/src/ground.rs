use las::{Read, Reader, point::Classification, Point};
use rand::{thread_rng, seq::SliceRandom};

pub fn reduce_ground(point_cloud: &mut Reader, ground_percentage: f64) -> Vec<Point> {
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