#include "geo.h"
#include <math.h>

const float earthR = 6371009;
const float degToRad = 0.01745329251994;//M_PI / 180;
const float radToDeg = 57.29577951308233;//180/M_PI
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

float Point::bearing(const Point* point) const
{
  /*
  http://www.movable-type.co.uk/scripts/latlong.html
  var φ1 = lat1.toRadians();
  var φ2 = lat2.toRadians();
  var Δφ = (lat2-lat1).toRadians();
  var Δλ = (lon2-lon1).toRadians();
  θ = atan2( sin Δλ ⋅ cos φ2 , cos φ1 ⋅ sin φ2 − sin φ1 ⋅ cos φ2 ⋅ cos Δλ )
  */
  float f1 = this->latitude * degToRad;
  float f2 = point->latitude * degToRad;
  float dl = (point->longitude - this->longitude) * degToRad;
  float y = sin(dl) * cos(f2);
  float x = cos(f1) * sin(f2) - sin(f1) * cos(f2) * cos(dl);
  float rad = atan2(y,x);
  return (rad > 0 ? rad : (2 * M_PI + rad)) * radToDeg;
}

float sqr(float v)
{
  return v * v;
}

float haversine(float f)
{
  //haversine(φ) = sin²(φ/2) = (1 - cos(φ)) / 2
  return sqr(sin(f * 0.5));
}

float Point::distance(const Point* point) const
{
  //Haversine formula from http://www.movable-type.co.uk/scripts/latlong.html
  //a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
  //c = 2 ⋅ atan2( √a, √(1−a) )
  //d = R ⋅ c
  float f1 = this->latitude * degToRad;
  float f2 = point->latitude * degToRad;
  float dl = (point->longitude - this->longitude) * degToRad;
  float df = f2 - f1;
  float a = haversine(df) + cos(f1) * cos(f2) * haversine(dl);
  float c = 2 * atan2(sqrt(a), sqrt(1.0 - a));
  return earthR * c;
}
