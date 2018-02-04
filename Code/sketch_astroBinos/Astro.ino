// Classe utilisée pour la communication au format LX2000
class Astro
{
  private:
    //float pitch, yaw, roll;
    char input[20];
    //Computed values for right ascention and declinaison
    double ra = 0;
    double de = 0;
    double lat, lng, lst;
    String Zeros(double num);
    double Declinaison(double az, double al, double latN);
    double LocalSideralTime(int YYYY, int MM, int DD, int hh, int mm, int ss, int lonE);
    double RightAscention(double az, double al, double latN, double de, double lst);

    
    // ToDO : Mettre à jour l'heure
    int uYYYY = 2018;
    int uMM = 02;
    int uDD = 3;
    int uhh = 21;
    int umm = 11;
    int uss = 0;
//    int lonE = -4.0979; //longitude Est
//    int latN = 47.9975; //latitude Nord

  public:
    void Setup(double lat, double lng);
    void Calc(float az, float al);
    void Communication();
    void Trace();
};

void Astro::Setup(double vlat, double vlng) {
  // Récupération coordonnées GPS
  lat = vlat; // latitude GPS
  lng = vlng; // longitude GPS
}

void Astro::Calc(float az, float al){
  lst = LocalSideralTime( uYYYY,  uMM, uDD, uhh, umm, uss, lon);
  de = Declinaison( az, al, lat);
  ra = RightAscention(az, al, lat, de, lst);
}

void Astro::Trace(){
  Serial.print("lst, de, ra ; ");
  Serial.print(lst);
  Serial.print(" ; ");
  Serial.print(de, 6);
  Serial.print(" ; ");
  Serial.println(ra, 6);
}
void Astro::Communication() {
  {
    int i = 0;
    input[i++] = Serial.read();
    delay(5);
    while ((input[i++] = Serial.read()) != '#') {
      delay(5);
    }

    digitalWrite(myLed, (state) ? HIGH : LOW);
    state = !state;

    input[i] = '\0';
    if (input[1] == ':' && input[2] == 'G' && input[3] == 'R' && input[4] == '#') {
      //Serial.print("05:34:24#");
      double h = int(ra);
      double m = int((abs(ra) - abs(int(ra))) * 60);
      double s = (abs(ra) - abs(h)) * 3600 - (m * 60);
      Serial.print(Zeros(h) + ":" + Zeros(m) + ":" + Zeros(s) + "#");
    }

    if (input[1] == ':' && input[2] == 'G' && input[3] == 'D' && input[4] == '#') {
      //Serial.print("-08*11#");
      double d = int(de);
      double mm = int((abs(de) - abs(int(de))) * 60);
      String deg = Zeros(d);
      if (d >= 0) {
        deg = "+" + deg;
      }
      Serial.print(deg + "*" + Zeros(mm) + "#");
    }
    if (input[1] == 'D' && input[2] == 'E' && input[3] == 'B' && input[4] == '#') {
      SerialDebug = true;
    }
    if (input[1] == 'N' && input[2] == 'O' && input[3] == 'D' && input[4] == '#') {
      SerialDebug = false;
    }
  }
}


///
/// Calculate declinaison given Altitude, Azimuth, Latitude N and Longitude E
/// Formulas obtained from http://129.79.46.40/~foxd/cdrom/musings/formulas/formulas.htm
///
/// az : Azimuth (0 from South growing when turning East) in decimal degrees
/// al : Altitude in decimal degrees
/// latN : Latitude North in decimal degrees
///
/// RETURNS : Declinaison in decimal degrees
///
double Astro::Declinaison(double az, double al, double latN)
{
  az = az * PI / 180.0; //conversion to radians
  al = al * PI / 180.0; //conversion to radians
  latN = latN * PI / 180.0; //conversion to radians

  double sinDe = (sin(al) * sin(latN)) + (cos(al) * cos(latN) * cos(az));

  double de = asin(sinDe);

  return de * 180.0 / PI; //conversion to decimal degrees
}

///
/// Calculate the right ascention given Azimuth, Altitude, Latitude N, Declinaison and Local Sideral Time
/// Formulas obtained from http://129.79.46.40/~foxd/cdrom/musings/formulas/formulas.htm
///
/// az : Azimuth (0 from South growing when turning East) in decimal degrees
/// al : Altitude in decimal degrees
/// latN : Latitude North in decimal degrees
/// de : Declinaison in decimal degrees
/// lst : Local sideral time in decimal hour
///
double Astro::RightAscention(double az, double al, double latN, double de, double lst)
{
  de = de * PI / 180; //conversion to radians
  az = az * PI / 180; //conversion to radians
  al = al * PI / 180; //conversion to radians
  latN = latN * PI / 180; //conversion to radians

  double cosHA = (sin(al) - (sin(latN) * sin(de))) / (cos(latN) * cos(de));
  //Serial.println("cosHA="+String(cosHA,15));
  //Serial.println("acos(cosHA)="+String(acos(cosHA),15));
  if (cosHA < -1) {
    cosHA = -1.0;
  }
  double ha = acos(cosHA);
  //Serial.println(ha);
  //Serial.println(sin(az));
  if (sin(az) > 0) {
    ha = 2 * PI - ha;
  }
  ha = ha / (2 * PI) * 24; //calculate the hour angle in decimal hours
  //Serial.println("ha="+String(ha));

  //Now we can convert the hour angle into Right Ascention
  double ra = lst - ha;
  if (ra < 0) {
    ra = ra + 24;
  }
  return ra; //this is the right ascention
}

///
/// Calcule le jour sideral en fonction de l'heure GMT et de la longitude
/// Formulas obtained from : http://129.79.46.40/~foxd/cdrom/musings/formulas/formulas.htm
///
/// YYYY : Year as integer (current gregorian calender)
/// MM : Month of the year as integer, starting at 1 for January
/// DD : Day of the month as integer starting at 1
/// hh : GMT hour as integer
/// mm : minutes as integer
/// ss : seconds as integer
/// lon : observers Longitude East as decimal degrees
///
/// RETURNS : Local sideral time in decimal hour
///
double Astro::LocalSideralTime(int YYYY, int MM, int DD, int hh, int mm, int ss, int lonE)
{
  //Calcul du jour Julien
  if (MM < 3) {
    MM = MM + 12;
    YYYY = YYYY - 1;
  }

  int A = int(YYYY / 100);
  int B = 2 - A + int(A / 4);
  int C = int(365.25 * YYYY);
  int E = int(30.6001 * (MM + 1));
  double JD0 = B + C + E + DD + 1720994.5; //Calculate JD (Julian Date) for the day on the previous midnight.

  //Serial.println("JD0="+String(JD0));

  //GMT sideral time
  double UT = hh + mm / 60 + ss / 3600; //GMT time in decimal hour
  double JD = JD0 + UT / 24;

  double D = JD - 2451545.0;
  //double D0 = JD0 - 2451545.0;
  //double T = (JD - 2451545.0) / 36525.0;
  //double TO = 6.697374558 + (2400.051336 * T) + (0.000025862 * T * T) + (UT * 1.0027379093);
  double TO = 18.697374558 + 24.06570982441908 * D;
  TO = fmod(TO, 24); //this is greenwhich sideral time in decimal hours

  //Serial.println("TO="+String(TO));

  //Local sideral time LST
  double LST = TO + (lonE / 15.0);
  LST = fmod(LST, 24); //this is local sideral time in decimal hours
  //Serial.println("(lonE / 15.0)="+String((lonE / 15.0)));
  //Serial.println("LST="+String(LST));

  return LST;
}

///
/// Returns the number as XX with a leading zero
///
String Astro::Zeros(double num) {
  if (int(abs(num)) > 9.0) {
    return String(int(num));
  }
  else if (int(num) < 0 && int(num) > -10) {
    return "-0" + String(abs(int(num)));
  }
  else {
    return "0" + String(int(num));
  }
}

