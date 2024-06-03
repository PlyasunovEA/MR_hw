#include "matcher.h"
#include <tf/transform_datatypes.h>

void Matcher::on_laser_scan(const sensor_msgs::LaserScan& scan)
{
  ROS_INFO_STREAM("on laser scan");
  detect_features(scan);  
  publish_features(scan.header);
  predict_features_poses();
  find_feature_pairs();
  find_transform();
  update_base_features();
  publish_transform(scan.header);  
}

void Matcher::publish_features(const std_msgs::Header& header)
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.color.a = 1.0;
  marker.color.g = 1.0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.points.resize(new_features.size());
  for (std::size_t i = 0; i < marker.points.size(); ++i) {
    marker.points[i].x = new_features[i].x();
    marker.points[i].y = new_features[i].y();
    marker.points[i].z = 0;
  }
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.id = 100;
  marker.ns = "features";
  feature_pub.publish(marker);
}

void Matcher::add_feature(const sensor_msgs::LaserScan& scan, std::size_t start, std::size_t finish) {
  // добавляем только особенные  точки на которые попало более 2 лучей
  if (finish - start < 2) {
  	return;
  }
  ROS_INFO_STREAM("Add feature between " << start << " " << finish);
  // TODO Здесь должен быть код определения координаты центра круглого препятствия 
  
  // Инициализация переменнх для хранения суммы координат и количества точек
  double x_sum = 0.0;
  double y_sum = 0.0;
  std::size_t count = 0;
  // Цикл по точкам лидара между индексами start и finish
  for (std::size_t i = start; i <= finish; ++i) {
    // Вычисляем преобразовние в декартовы координаты и угол
    double angle = scan.angle_min + i * scan.angle_increment;
    double x = scan.ranges[i] * std::cos(angle);
    double y = scan.ranges[i] * std::sin(angle);

    // Производим суммирование координат
    x_sum += x;
    y_sum += y;
    count++;
  }
  
  // Вычисляем усреднённые координаты центр круглого столба
  double obstacle_x = x_sum / count;
  double obstacle_y = y_sum / count;

  // добавляем в вектор особенных точек
  new_features.push_back(Eigen::Vector2d(obstacle_x, obstacle_y));
}

void Matcher::detect_features(const sensor_msgs::LaserScan& scan)
{
  new_features.clear();
  // TODO Здесь должен быть код для пределения особенные точек скана
  // В цикле по лучам ищем начальный и конечный индексы лучей, падающих на одно препятствие
  // и вызываем add_feature
  const std::vector<float>& ranges = scan.ranges;
  const float angle_increment = scan.angle_increment;
  // Шагаем циклом по лучам с лидара и ищем локальные минимуны, которые считаем за центр столбов
  for (std::size_t i = 1; i < ranges.size() - 1; ++i) {
    if (ranges[i] < ranges[i - 1] && ranges[i] < ranges[i + 1]) {
      // Вычисляем угол и преобразование в декартовы координаты
      double angle = scan.angle_min + i * angle_increment;
      double x = ranges[i] * std::cos(angle);
      double y = ranges[i] * std::sin(angle);
      
      // Добавляем обнаруженную особую точку
      new_features.push_back(Eigen::Vector2d(x, y));
    }
  }

}

void Matcher::predict_features_poses() {
  const Eigen::Isometry2d interpolated_transform = incremental_transform * transform;
  predicted_features.resize(new_features.size());
  // TODO здесь должен быть код пересчета new_features в predicted_features с помощью interpolated_transform
  // Цикл по новым особым точкам для предсказани их положения после перемещения робота
  for (std::size_t i = 0; i < new_features.size(); ++i) {
    // Получаем новое положение точки с помощью интерполированной матрицы перехода
    predicted_features[i] = interpolated_transform * new_features[i];
  }
}

void Matcher::find_feature_pairs() {
  feature_pair_indices.clear();
  for (std::size_t base_index = 0; base_index < base_features.size(); ++base_index) {
  	std::size_t nearest_index = 0;
  	double nearest_distance = 1000000;

  	for (std::size_t new_index = 0; new_index < predicted_features.size(); ++new_index) {
  	  const double distance = (base_features[base_index] - predicted_features[new_index]).norm();
  	  if (distance < nearest_distance) {
  	  	nearest_index = new_index;
  	  	nearest_distance = distance;
  	  }
  	}
  	ROS_INFO_STREAM("base_index " << base_index << " nearest " << nearest_index << " d = " << nearest_distance);
  	
  	if ((nearest_distance < 100.0) && (nearest_distance > 0.5)) 
  	{
      		feature_pair_indices.push_back(nearest_index);
  	} 
  	else 
  	{
  	  	feature_pair_indices.push_back(-1);
  	}

  }
}

void Matcher::find_transform()
{
	Eigen::Matrix2d W = Eigen::Matrix2d::Zero();
	Eigen::Vector2d base_center = Eigen::Vector2d::Zero();
	Eigen::Vector2d new_center = Eigen::Vector2d::Zero();
	std::size_t pairs = 0;
	// определяем количество пар и вычисляем геом. центр
	for (std::size_t i = 0; i < feature_pair_indices.size(); ++i) {
	  if (feature_pair_indices[i] >= 0){
	  	++pairs;
	  	base_center += base_features[i];
	  	new_center += predicted_features[feature_pair_indices[i]];
	  }       
	}
	ROS_INFO_STREAM("pairs = " << pairs);
	if (pairs < 2) {
		ROS_ERROR_STREAM("Not enaugh feature pairs!!! ");
		return;
	}
	base_center /= pairs;
	new_center /= pairs;
	// вычисляем матрицу W
	for (std::size_t i = 0; i < feature_pair_indices.size(); ++i) {
	  if (feature_pair_indices[i] >= 0){
	  	W += (predicted_features[feature_pair_indices[i]] - new_center) * (base_features[i] - base_center).transpose();
	  }       
	}
	// вычисление угла из svd разложения сводится к
	float angle = atan2 ((W (0, 1) - W (1, 0)), (W(0, 0) + W (1, 1)));
	Eigen::Matrix2d R;
	R (0, 0) = R (1, 1) = cos (angle);
  	R (0, 1) = -sin (angle);
  	R (1, 0) = sin (angle);
	Eigen::Vector2d t = base_center - R * new_center;
	Eigen::Isometry2d result = Eigen::Translation2d(t) * Eigen::Isometry2d(R);

	// обновляем трансформы
	incremental_transform = result * incremental_transform;
	transform = incremental_transform * transform;
	// вычисляем среднюю ошибку
	double err = 0;
	for (std::size_t i = 0; i < feature_pair_indices.size(); ++i) {
	  if (feature_pair_indices[i] >= 0) {
	  	err += (base_features[i] - transform * new_features[feature_pair_indices[i]]).norm();
	  }       
	}
	ROS_INFO_STREAM("Mean error = " << err/pairs);
	ROS_INFO_STREAM("Incremental transform " << std::endl << incremental_transform.matrix());
	ROS_INFO_STREAM("Transform to initial pose " << std::endl << transform.matrix());
}

void Matcher::publish_transform(const std_msgs::Header& header)
{
	// публикуем одометрию
  nav_msgs::Odometry odo;
  odo.header.stamp = header.stamp;
  odo.header.frame_id = map_frame;
  odo.child_frame_id = header.frame_id;

  const auto& matrix = transform.matrix();

  double yaw = atan2(matrix(1, 0), matrix(0, 0));
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  tf::quaternionTFToMsg(q, odo.pose.pose.orientation);
  odo.pose.pose.position.x = transform.translation().x();
  odo.pose.pose.position.y = transform.translation().y();
  // вычисляем скорости
  const auto& inc_matrix = incremental_transform.matrix();
  double dt = (header.stamp - last_stamp).toSec();
  last_stamp = header.stamp;
  double omega = atan2(matrix(1, 0), matrix(0, 0)) / dt;
  odo.twist.twist.angular.z = omega;
  odo.twist.twist.linear.x = incremental_transform.translation().x() / dt;
  odo.twist.twist.linear.y = incremental_transform.translation().y() / dt;
  odo_pub.publish(odo);
  
  // публикуем трансформ от скана до карты, 
  // не наоборот, так как дерево tf - однонаправленное и в нем уже есть основание - система odom
  tf::Transform tf_transform;
  Eigen::Isometry2d inverted_transform = transform.inverse();
  tf_transform.setOrigin( tf::Vector3(inverted_transform.translation().x(), 
  									  inverted_transform.translation().y(), 
  									  0.0) );
  tf::Quaternion inv_q;
  const auto& inv_matrix = inverted_transform.matrix();
  double inv_yaw = atan2(inv_matrix(1, 0), inv_matrix(0, 0));
  inv_q.setRPY(0, 0, inv_yaw);
  tf_transform.setRotation(inv_q);
  br.sendTransform(tf::StampedTransform(tf_transform, header.stamp, header.frame_id, map_frame));
}

void Matcher::update_base_features()
{
	// обновляем особенные точки если их не было - для первого измерения
  if (base_features.empty()) {
  // Чистим индексы пар особенных точек и присваиваем базовым точкам значения новых точек
	feature_pair_indices.clear();
	base_features = new_features;	
  } else {
  	// TODO здесь должен быть код обновления опорных особенных точек по определенным условиям
    // Далее просто сравниваем расстояния и обновляем базовые точки, если новые ближе всего к роботу
  	for (std::size_t i = 0; i < new_features.size(); ++i) {
      // Вычислям расстояние от текущей позиции робота до базовой и новой особой точки 
      double distance_base = (base_features[i] - transform.translation()).norm();
      double distance_new = (new_features[i] - transform.translation()).norm();
      
      // Если новая точка ближе всего к роботу, то обновляем базовую точку
      if (distance_new < distance_base) {
        base_features[i] = new_features[i];
      }
    }
  }
  // Выводим информаццию о базовых особых точках в консоль
  ROS_INFO_STREAM("Features");
  for (const auto& feature : base_features) {
    ROS_INFO_STREAM(feature.transpose());
  }
}
