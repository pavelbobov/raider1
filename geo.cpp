#include "geo.h"
#include <math.h>

const float earthR = 6371009;
Point::Point() 
{
  set(0, 0);
}

Point::Point(float lat, float lon) 
{
  set(lat, lon);
}

void Point::set(float lat, float lon) 
{
  this->latitude = lat;
  this->longitude = lon;
}

float Point::bearing(Point* point)
{
  /*
  http://www.movable-type.co.uk/scripts/latlong.html
  var φ1 = lat1.toRadians();
  var φ2 = lat2.toRadians();
  var Δφ = (lat2-lat1).toRadians();
  var Δλ = (lon2-lon1).toRadians();
  θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
  */
  float f1 = this->latitude / 180.0 * M_PI;
  float f2 = point->latitude / 180.0 * M_PI;
  float dl = (point->longitude - this->longitude) / 180.0 * M_PI;
  float y = sin(dl) * cos(f2);
  float x = cos(f1) * sin(f2) - sin(f1) * cos(f2) * cos(dl);
  float rad = atan2(y,x);
  return (rad > 0 ? rad : (2 * M_PI + rad)) * 180.0 / M_PI;
}

float Point::bearing(Point* point)
{
  /*
  http://www.movable-type.co.uk/scripts/latlong.html
  var φ1 = lat1.toRadians();
  var φ2 = lat2.toRadians();
  var Δφ = (lat2-lat1).toRadians();
  var Δλ = (lon2-lon1).toRadians();
  θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
  */
  float f1 = this->latitude / 180.0 * M_PI;
  float f2 = point->latitude / 180.0 * M_PI;
  float dl = (point->longitude - this->longitude) / 180.0 * M_PI;
  float y = sin(dl) * cos(f2);
  float x = cos(f1) * sin(f2) - sin(f1) * cos(f2) * cos(dl);
  float rad = atan2(y,x);
  return (rad > 0 ? rad : (2 * M_PI + rad)) * 180.0 / M_PI;
}

float Point::distance(const Point* point) const
{
  float f1 = this->latitude / 180.0 * M_PI;
  float f2 = point->latitude / 180.0 * M_PI;
  float dl = (point->longitude - this->longitude) / 180.0 * M_PI;
  float df = f2 - f1;
  float a = sq(sin(df * 0.5)) + cos(f1) * cos(f2) * sq(sin(dl*0.5));
  float c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
  return earthR * c;
}
