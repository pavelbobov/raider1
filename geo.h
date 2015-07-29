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
  float bearing(Point* point);  
};
