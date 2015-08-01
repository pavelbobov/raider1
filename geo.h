class Point
{
  public:
  //Geographic coordinates in decimal degrees
  float latitude; 
  float longitude;

  Point();
  Point(float lat, float lon);
  
  void set(float lat, float lon) ;
  
  //Computes bearing from this to the specified point
  float bearing(const Point* point) const;
  //Computes distance from this to the specified point in meters (accuracy %0.5)
  float distance(const Point* point) const;
};
