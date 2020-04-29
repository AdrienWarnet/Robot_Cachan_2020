using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace TrajectoryGenerator
{
    public class TrajectoryGenerator
    {
        //L'expression de la distance de freinage est V(t)^2 / 2a
        private double fEchAsserv;

        public LocationRobot2Roues robotLocation = new LocationRobot2Roues(0, 0, 0, 0, 0);
        public LocationRobot2Roues ghostLocation = new LocationRobot2Roues(0, 0, 0, 0, 0);
        public LocationRobot2Roues wayPointLocation = new LocationRobot2Roues(0, 0, 0, 0, 0);

        public TrajectoryGenerator(double fe)
        {
            fEchAsserv = fe;
            wayPointLocation = new LocationRobot2Roues(0, 0, 0, 0, 0);
            ghostLocation = new LocationRobot2Roues(0, 0, 0, 0, 0);
            robotLocation = new LocationRobot2Roues(0, 0, 0, 0, 0);
        }

        public void SetCurrentWayPoint(PointD coordonnees)
        {
            wayPointLocation.X = coordonnees.X;
            wayPointLocation.Y = coordonnees.Y;
            integraleEcartAngleMax = KpAngleTheta / KiAngleTheta * 0.1;
            integraleEcartDistMax = KpLin / KiLin * 0.1;
        }

        double vitesseAngulaireMax = 3.0;//5;//en rad.s-1
        double accelerationAngulaireCourante = 8.0;
        double KpAngleTheta = 10.0;
        double KiAngleTheta = 0.5;
        double integraleEcartAngle = 0;
        double integraleEcartAngleMax = 0;

        double vitesseLineaireMax = 0.5;//0.3;//en m.s-1
        double accelerationLineaireCourante = 1.0;
        double KpLin = 10.0;
        double KiLin = 0.1;
        double integraleEcartDist = 0;
        double integraleEcartDistMax = 0;
        double distanceCourante;


        double alpha = 0.005;



        public float [] GenerateSpeedConsigne(PointD position, double vitesseLineaireCourante, double angleRobotRefTerrain)
        {
            //Calcul de l'angle de destination
            double angleDestinationRefGhost;
            angleDestinationRefGhost = Math.Atan2(wayPointLocation.Y - ghostLocation.Y, wayPointLocation.X - ghostLocation.X);
            
            //Calcul de la distance vers la cible
            double ecartDistanceDestinationGhost = Math.Sqrt(Math.Pow(wayPointLocation.X - ghostLocation.X, 2) + Math.Pow(wayPointLocation.Y - ghostLocation.Y, 2));

            //On replace les angles destination ref terrain et consigne courant dans un intervalle de -PI; PI autour de l'angle réel du robot.
            double ecartCapGhost = Toolbox.ModuloByAngle(ghostLocation.Theta, angleDestinationRefGhost - ghostLocation.Theta);

            //Calcul de l'erreur d'angle entre l'angle de consigne instantané et la cible
            ecartCapGhost = Toolbox.Modulo2PiAngleRad(ecartCapGhost);

            double disRobotToGhost = Math.Sqrt(Math.Pow(ghostLocation.X - position.X, 2) + Math.Pow(ghostLocation.Y - position.Y, 2));

            if (disRobotToGhost > 0.03)
            {
                ghostLocation.X = ghostLocation.X * (1 - alpha) + alpha * position.X;
                ghostLocation.Y = ghostLocation.Y * (1 - alpha) + alpha * position.Y;
            }

            //Calcul de l'angle de freinage angulaire
            double angleFreinageAngulaireGhost = Math.Pow(ghostLocation.Vtheta, 2) / (2 * accelerationAngulaireCourante);


            //Rampe de vitesse Angulaire
            if (Math.Abs(ecartCapGhost) > angleFreinageAngulaireGhost)
            {
                //Phase d'accélération/vitesse constante
                if (ecartCapGhost >= 0)
                    ghostLocation.Vtheta = Math.Min(ghostLocation.Vtheta + accelerationAngulaireCourante / fEchAsserv, vitesseAngulaireMax);
                else
                    ghostLocation.Vtheta = Math.Max(ghostLocation.Vtheta - accelerationAngulaireCourante / fEchAsserv, -vitesseAngulaireMax);
            }
            else
            {
                //Phase de décélération
                if (ecartCapGhost >= 0)
                    ghostLocation.Vtheta = Math.Max(ghostLocation.Vtheta - accelerationAngulaireCourante / fEchAsserv, 0);
                else if (ecartCapGhost < 0)
                    ghostLocation.Vtheta = Math.Min(ghostLocation.Vtheta + accelerationAngulaireCourante / fEchAsserv, 0);   

            }

            //Recalcul de la position angulaire théorique courante
            ghostLocation.Theta += ghostLocation.Vtheta / fEchAsserv;

            //-----------------------------------------------------Phase linéaire-------------------------------------------------------------
            //calcul distance de freinage linéaire
            double distanceFreinageGhost = Math.Pow(ghostLocation.Vlin, 2) / (2 * accelerationLineaireCourante);                
            double correctionLineaire = 0;

            if (Math.Abs(ecartCapGhost) <= 0.005)
            {
                if (ecartDistanceDestinationGhost > 0.02)
                {
                    if (ecartDistanceDestinationGhost > distanceFreinageGhost)
                    {
                        //Phase d'accélération/vitesse constante
                        ghostLocation.Vlin = Math.Min(ghostLocation.Vlin + accelerationLineaireCourante / fEchAsserv, vitesseLineaireMax);
                    }
                    else
                    {
                        //Phase de décélération
                        ghostLocation.Vlin = Math.Max(ghostLocation.Vlin - accelerationLineaireCourante / fEchAsserv, 0);
                    }
                } 
                //else 
                //    if(ecartDistanceDestinationGhost <= 0.02)
                //    {
                //        ghostLocation.Vlin = 0;
                //        ghostLocation.Vtheta = 0;
                //    }
                    
                    //////Recalcul de la position lineaire théorique 
                    ghostLocation.distanceParcourue += ghostLocation.Vlin / fEchAsserv;

            }
            else
            {
                ghostLocation.Vlin = Math.Max(ghostLocation.Vlin - accelerationLineaireCourante / fEchAsserv, 0);
            }

            //Si la vitesse Obtenue est faible et que l'on est très proche du WayPoint, on la réduit à 0 pour éviter les micro-mouvements

            if (ecartDistanceDestinationGhost <= 0.02)
            {
                ghostLocation.Vlin = 0;
                ghostLocation.Vtheta = 0;
            }

            ghostLocation.X += ghostLocation.Vlin * Math.Cos(ghostLocation.Theta) / fEchAsserv;
            ghostLocation.Y += ghostLocation.Vlin * Math.Sin(ghostLocation.Theta) / fEchAsserv;

            float[] vitesseTab = new float[2] { 0, 0 };

            vitesseTab =  AsservissementPosition(vitesseLineaireCourante, angleRobotRefTerrain, ecartDistanceDestinationGhost);

            return vitesseTab;
        }

        public float[] AsservissementPosition(double vitesseLineaireCourante, double angleRobotRefTerrain, double ecartDistanceDestinationGhost)
        {
           
            //-------------------------------------------------Asservisement en angulaire----------------------------------------------------
            //Asservissement en angle sur le ghost
            double ecartAngle = Toolbox.ModuloByAngle(ghostLocation.Theta, ghostLocation.Theta - angleRobotRefTerrain);
            ecartAngle = Toolbox.Modulo2PiAngleRad(ecartAngle);
            double correctionAngulaireKp = KpAngleTheta * ecartAngle;
            integraleEcartAngle += ecartAngle / fEchAsserv;
            integraleEcartAngle = Toolbox.LimitToInterval(integraleEcartAngle, -integraleEcartAngleMax, integraleEcartAngleMax);
            double correctionAngulaireKi = KiAngleTheta * integraleEcartAngle;
            double correctionAngulaire = correctionAngulaireKp + correctionAngulaireKi;


            //-------------------------------------------------Asservisement en Linéaire-----------------------------------------------------

            double correctionLineaire = 0;
            ////Recalcul de la position lineaire théorique courante
            distanceCourante += vitesseLineaireCourante / fEchAsserv;
            double ecartDistanceCourante = ghostLocation.distanceParcourue - distanceCourante;
           

            //Asservissement en linéaire
            double correctionLineaireKp = KpLin * ecartDistanceCourante;
            integraleEcartDist += ecartDistanceCourante / fEchAsserv;
            integraleEcartDist = Toolbox.LimitToInterval(integraleEcartDist, -integraleEcartDistMax, integraleEcartDistMax);
            double correctionLineraireKi = KiLin * integraleEcartDist;
            correctionLineaire = correctionLineaireKp + correctionLineraireKi;

            if (Math.Abs(ecartDistanceDestinationGhost) < 0.02)
            {
                correctionAngulaire = 0;
                correctionLineaire = 0;
            }


            float[] vitesseTab = new float[2] { 0, 0 };
            vitesseTab[0] = (float)correctionAngulaire;
            vitesseTab[1] = (float)correctionLineaire;
            return vitesseTab;

        }




           
    }
}

