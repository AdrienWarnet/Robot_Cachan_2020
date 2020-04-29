using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using GeometryElements;
using Lidar;
using Utilities;

namespace WpfApp1
{
    class Robot
    {
        public List<double> RawLidarPointsXList = new List<double>();
        public List<double> RawLidarPointsYList = new List<double>();

        public List<LidarPoint> FilteredLidarPointList = new List<LidarPoint>();
        public List<double> FilteredLidarPointsXList = new List<double>();
        public List<double> FilteredLidarPointsYList = new List<double>();
        public bool flagLidarUpdated = false;

        public List<RobotLidarObject> objectList = new List<RobotLidarObject>();
        public List<RobotLidarObject> ballList = new List<RobotLidarObject>();
        public List<double> ballListXPosition = new List<double>();
        public bool flagBallListUpdated = false;

        public double seuilFiltrageDist = 0.015;

        public int timestamp;
        public double positionXOdo;
        public double positionYOdo;
        public PointD locationRefterrain;
        public double angle;
        public double vitesseLineaire;
        public double vitesseAngulaire;
        public double vitesseLineaireX;
        public double vitesseLineaireY;

        public double consigneMoteurGauche;
        public double consigneMoteurDroit;

        public double consigneVitesseLineaire;
        public double consigneVitesseAngulaire;

        public bool flagOdoIsUpdated = false;
    }

    class RobotLidarObject
    {
        public List<LidarPoint> Points = new List<LidarPoint>();
        public double angle;
        public double largeur;
        public double distance;
        public double xObject;
        public double yObject;

        public void CharacterizeObject()
        {
            angle = Points.Average(r => r.Angle);
            distance = Points.Average(r => r.Distance);
            largeur = distance*(Points.Max(r => r.Angle)- Points.Min(r => r.Angle));
            xObject = distance * Math.Cos(angle);
            yObject = distance * Math.Sin(angle);
        }
    }
}
