#region Lib
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Forms ;
using System.IO.Ports;
using WpfWorldMapDisplay;
using System.Windows.Threading;
using ExtendedSerialPort;
using Lidar;
using Utilities;
using GeometryElements;
using System.Media;
using SciChart.Charting.Visuals;
using AdvancedTimers;
using System.Diagnostics;

#endregion

namespace WpfApp1
{
    /// <summary>
    /// Logique d'interaction pour MainWindow.xaml
    /// </summary>
    //TODO : inverser les axes x et y 
    //TODO : toute les 2,04 min/124000timestamp tout ce remet a zero re=égler se probleme
    //TODO : régler problème rampe accel et deccel des vitesse linéaire trop brusque
    //TODO : vérifier le récup ball avec le changement de repère si ça fonctionne
    //TODO : débugger jack qui ne fonctionne plus et mettre la séquence du carrée sur le jack 

    public partial class MainWindow : Window
    {

        #region Initialisation
        //serial port et dispatcherTimer
        ReliableSerialPort serialPort;
        DispatcherTimer timerLidar = new DispatcherTimer(DispatcherPriority.Send);
        DispatcherTimer timerOdo = new DispatcherTimer(DispatcherPriority.Send);
        DispatcherTimer timerCarre = new DispatcherTimer(DispatcherPriority.Send);

        //timer haute fréquence en hz 
        HighFreqTimer timerTraj; 

       Queue<byte> byteListReceived = new Queue<byte>();

        //lidar
        Robot robot = new Robot();
        RobotLidarObject ball = new RobotLidarObject();
        //double fe=50;
        
        SickLidar robotLidar = new SickLidar(18340986);// 18340986

        //ajout de son
        //SoundPlayer depart = new SoundPlayer("C:\\Users\\LattePanda\\Desktop\\Son_match\\le_départ.wav");

        TrajectoryGenerator.TrajectoryGenerator trajectoryGenerator;
        

        #endregion

        #region MainWindow
        public MainWindow()
        {
            trajectoryGenerator = new TrajectoryGenerator.TrajectoryGenerator(50);
            SciChartSurface.SetRuntimeLicenseKey(@"<LicenseContract>
            <Customer>Universite De Toulon</Customer>
            <OrderId>EDUCATIONAL-USE-0128</OrderId>
            <LicenseCount>1</LicenseCount>
            <IsTrialLicense>false</IsTrialLicense>
            <SupportExpires>02/17/2020 00:00:00</SupportExpires>
            <ProductCode>SC-WPF-2D-PRO-SITE</ProductCode>
            <KeyCode>lwAAAQEAAACS9FAFUqnVAXkAQ3VzdG9tZXI9VW5pdmVyc2l0ZSBEZSBUb3Vsb247T3JkZXJJZD1FRFVDQVRJT05BTC1VU0UtMDEyODtTdWJzY3JpcHRpb25WYWxpZFRvPTE3LUZlYi0yMDIwO1Byb2R1Y3RDb2RlPVNDLVdQRi0yRC1QUk8tU0lURYcbnXYui4rna7TqbkEmUz1V7oD1EwrO3FhU179M9GNhkL/nkD/SUjwJ/46hJZ31CQ==</KeyCode>
            </LicenseContract>");
            

            InitializeComponent();
            serialPort = new ReliableSerialPort("COM8", 115200, Parity.None, 8, StopBits.One);
            serialPort.Open();
            serialPort.DataReceived += SerialPort_DataReceived;
            checkBoxAuto.IsChecked = true;

            byte[] msgAllumeLedBleueStart = new byte[2];
            msgAllumeLedBleueStart[0] = 2;
            msgAllumeLedBleueStart[1] = 1;

            msgAllumeLedBleueStart = Message.UartEncodeMessage((int)commandRobot.ReglageLed, msgAllumeLedBleueStart.Length, msgAllumeLedBleueStart);
            serialPort.Write(msgAllumeLedBleueStart, 0, msgAllumeLedBleueStart.Length);

            //timer lidar
            timerLidar.Interval = TimeSpan.FromMilliseconds(50); //10Hz
            timerLidar.Tick += TimerLidar_Tick;
            timerLidar.Start();


            //timer trajectory haute fréq
            timerTraj = new HighFreqTimer(50); //en hz
            timerTraj.Tick += TimerTraj_Tick;
            timerTraj.Start();

            //timer Odo 
            timerOdo.Interval = TimeSpan.FromMilliseconds(30); //50hz
            timerOdo.Tick += TimerOdo_Tick;
            timerOdo.Start();

            //timer de séquence carrée 
            timerCarre.Interval = TimeSpan.FromMilliseconds(20);
            timerCarre.Tick += TimerCarre_Tick;

            osciloPosition.SetTitle("Graphique Oscilloscope");

            osciloPosition.AddOrUpdateLine(0, 200, "angle");
            osciloPosition.ChangeLineColor(0, Colors.Aquamarine);
            osciloPosition.AddOrUpdateLine(1, 200, "consigne angle");
            osciloPosition.ChangeLineColor(1, Colors.BlanchedAlmond);

            osciloVitesseAngulaire.AddOrUpdateLine(0, 200, "consigne vitesse angulaire");
            osciloVitesseAngulaire.ChangeLineColor(0, Colors.YellowGreen);
            osciloVitesseAngulaire.AddOrUpdateLine(1, 200, "vitesse angulaire");
            osciloVitesseAngulaire.ChangeLineColor(1, Colors.AntiqueWhite);
            osciloVitesseAngulaire.AddOrUpdateLine(2, 200, "vitesse angulaire Ghost");
            osciloVitesseAngulaire.ChangeLineColor(2, Colors.Blue);

            osciloVitesseLineaire.AddOrUpdateLine(0, 200, "consigne vitesse linéaire");
            osciloVitesseLineaire.ChangeLineColor(0, Colors.CornflowerBlue);
            osciloVitesseLineaire.AddOrUpdateLine(1, 200, "vitesse linéaire");
            osciloVitesseLineaire.ChangeLineColor(1, Colors.DarkSalmon);
            osciloVitesseLineaire.AddOrUpdateLine(2, 200, "vitesse lineaire Ghost");
            osciloVitesseLineaire.ChangeLineColor(2, Colors.Red);

            //réglage lidar
            robotLidar.AngleMax = 120;
            robotLidar.AngleMin = -120;
            robotLidar.IsUpsideDown = true;
            robotLidar.Start();
            robotLidar.PointsAvailable += RobotLidar_PointsAvailable;
         
           

           

        }


        #endregion

        #region Lidar_PointsAvailable
        //On récupere les points du lidar, et on les convertit en mètre à partir des coordonnées polaires
        //On affiche ces points sur le graphique ZedGraph
        private void RobotLidar_PointsAvailable(object sender, LidarPointsReadyEventArgs e)
        {
            //Les données du Lidar sont en polaire (teta et dist) il faut les convertir en cartésien (x et y avec cos et sin)
            //grapheLidar.PointListClear("Lidar Data");  
            lock (robot)
            {
                robot.RawLidarPointsXList = robotLidar.LidarPoints.Select(r => r.Distance * Math.Cos(r.Angle)).ToList();
                robot.RawLidarPointsYList = robotLidar.LidarPoints.Select(r => r.Distance * Math.Sin(r.Angle)).ToList();

                robot.FilteredLidarPointList.Clear();

                //préfiltrage on suprime les point qui sont incohérent
                for (int a = 1; a < robotLidar.LidarPoints.Count - 1; a++)
                {
                    if (Math.Abs(robotLidar.LidarPoints[a].Distance - robotLidar.LidarPoints[a - 1].Distance) < robot.seuilFiltrageDist ||
                       Math.Abs(robotLidar.LidarPoints[a].Distance - robotLidar.LidarPoints[a + 1].Distance) < robot.seuilFiltrageDist)
                        robot.FilteredLidarPointList.Add(robotLidar.LidarPoints[a]);
                }

                //Calcul des points en cartésien

                robot.FilteredLidarPointsXList = robot.FilteredLidarPointList.Select(r => r.Distance * Math.Cos(r.Angle)).ToList();
                robot.FilteredLidarPointsYList = robot.FilteredLidarPointList.Select(r => r.Distance * Math.Sin(r.Angle)).ToList();

                for (int i = 0; i < robot.FilteredLidarPointsXList.Count(); i++)
                {

                    double x;
                    double y;
                    y = robot.FilteredLidarPointsYList[i];
                    x = robot.FilteredLidarPointsXList[i];

                    robot.FilteredLidarPointsXList[i] = (x * Math.Cos(robot.angle) - y * Math.Sin(robot.angle)) + robot.positionXOdo;
                    robot.FilteredLidarPointsYList[i] = (x * Math.Sin(robot.angle) + y * Math.Cos(robot.angle)) + robot.positionYOdo;
                }

            }
            robot.flagLidarUpdated = true;

            Ball_Detection(); //On envoie les coordonnées polaire et X et Y en mètre
      }
        #endregion

        #region Detection de Balle 

        int jackState;

        //---------------- Variable correcteur -------------
        double integ = 0, vitesseDeCorrection = 0;
        float vitesseLineaire = 0;
        float vitesseAngulaire = 0;
        double Kp = 0.2;
        int flagTurn = 0;

        StateToBalls stateRecupBall = StateToBalls.Waiting;


        //---------------------------------------------------------------------DETECTION DE BALLES---------------------------------------------
        private void Ball_Detection()// avant PointList pointsLidar
        {
            bool detectedObject = false;
            RobotLidarObject currentObject = new RobotLidarObject();

           
            robot.objectList.Clear();

            for (int j = 1; j < robot.FilteredLidarPointList.Count - 1; j++)
            {
                if ((robot.FilteredLidarPointList[j - 1].Distance - robot.FilteredLidarPointList[j].Distance) > 0.02)//avant (oldDistance - point.D)
                {
                    detectedObject = true;
                    currentObject = new RobotLidarObject();
                    currentObject.Points.Add(robot.FilteredLidarPointList[j]);
                }
                else if ((robot.FilteredLidarPointList[j].Distance - robot.FilteredLidarPointList[j - 1].Distance) > 0.02) //avant (point.D - oldDistance)
                {
                    //On a une fin d'objet
                    if (detectedObject == true)
                    {
                        //On a un objet qui se termine, on l'ajoute-------------------------------
                        currentObject.CharacterizeObject();
                        robot.objectList.Add(currentObject);
                        detectedObject = false;
                    }
                }
                else
                {
                    if (detectedObject == true)
                        currentObject.Points.Add(robot.FilteredLidarPointList[j]);
                }
            }

            // On filtre les objets
            robot.ballList = robot.objectList.Where(o => o.largeur > 0.02 && o.largeur <= 0.1).ToList();

            robot.ballListXPosition = robot.ballList.Select(r=>r.xObject).ToList();
            


            if (robot.ballList.Count() > 0) {               
                auto = 0;
                robot.flagBallListUpdated = true ; 
                //Recup_Balle();

            }
            else { 
                auto = 1;
            }

            byte[] autoValue = new byte[2];
            autoValue[0] = (byte)auto;
            autoValue[1] = (byte)150;
            autoValue = Message.UartEncodeMessage((int)commandRobot.RobotAutoControl, autoValue.Length, autoValue);
            serialPort.Write(autoValue, 0, autoValue.Length);
        }
        #endregion

        #region RecupBalls
        private void Recup_Balle()
        {
            double distBallePlusProche;
            double angleBallePlusProche;

            distBallePlusProche = robot.ballList.Min(r => r.distance);
            var balleProche = robot.ballList.Where(r => r.distance == distBallePlusProche).ToList();
            angleBallePlusProche = balleProche[0].angle;

            byte[] vitesseGoBall = new byte[2];
            byte[] msgBalle = new byte[8];

            #region Correcteur           
            //--------------------- CORRECTEUR ----------------------------

            //vitesseDeCorrection = Math.Abs(Kp * angleBallePlusProche);
            //integ += Math.Abs(Ki * angleBallePlusProche);
            //integ = Toolbox.LimitToInterval(integ, 15, 0);
            //vitesseDeCorrection += integ;
            #endregion

            #region Machine à état récuperrage de Balles

            switch (stateRecupBall)
            {
                case StateToBalls.Waiting:

                    vitesseDeCorrection = Kp * angleBallePlusProche;
                    vitesseDeCorrection = Toolbox.LimitToInterval(vitesseDeCorrection, 0.6, -0.6);
                    
                    //integ += Ki * angleBallePlusProche * 15;
                    ////integ = Toolbox.LimitToInterval(integ, 5, -5);
                    //vitesseDeCorrection += integ;

                    if (distBallePlusProche > 0.05)
                    {
                        if (angleBallePlusProche < Toolbox.DegToRad(5) && angleBallePlusProche > Toolbox.DegToRad(-5))
                        {
                            //flagCompteur = 1;
                            stateRecupBall = StateToBalls.FrontOf;
                        }
                        else
                            stateRecupBall = StateToBalls.Turn;
                    }
                    else
                        stateRecupBall = StateToBalls.Stop;

                    break;
                case StateToBalls.Turn:

                    flagTurn = 1;

                    if (distBallePlusProche <= 0.05)
                    {
                        stateRecupBall = StateToBalls.Stop;
                    }
                    else
                    {
                        if (angleBallePlusProche < 0) //Balle à droite --> tourne à droite, vitesse négative moteur droit
                        {
                            vitesseAngulaire = (float)(-0.4);
                            vitesseLineaire = 0;
                        }
                        else
                            if (angleBallePlusProche > 0)
                            {
                                vitesseAngulaire = (float)0.4;
                            vitesseLineaire = 0;
                            }

                        UartSendVitesseConsigne(vitesseAngulaire, vitesseLineaire);
                    }

                    stateRecupBall = StateToBalls.Waiting;

                    break;
                case StateToBalls.FrontOf:

                    if (flagTurn == 1)
                    {
                        //integ = 0;
                        flagTurn = 0;
                    }

                    if (distBallePlusProche <= 0.05)
                    {
                        stateRecupBall = StateToBalls.Stop;
                    }
                    else
                    {
                        vitesseAngulaire = 0;
                        vitesseLineaire = (float)0.7;

                        UartSendVitesseConsigne(vitesseAngulaire, vitesseLineaire);

                        stateRecupBall = StateToBalls.Waiting;
                    }

                    break;

                case StateToBalls.Stop:

                    vitesseAngulaire = 0;
                    vitesseLineaire = 0;

                    UartSendVitesseConsigne(vitesseAngulaire, vitesseLineaire);

                    if(distBallePlusProche > 0.05)
                    {
                        stateRecupBall = StateToBalls.Waiting;
                    }

                    break;
            }
            
            #endregion

        }
        #endregion

        #region UartSendVitesseConsigne
        private void UartSendVitesseConsigne(float consigneAngulaire, float consigneLineaire)
        {
            byte[] byteVAng = consigneAngulaire.GetBytes();
            byte[] byteVLin = consigneLineaire.GetBytes();

            byte[] msg = new byte[8];

            msg.SetValueRange(byteVAng,0,4);
            msg.SetValueRange(byteVLin,4,4);


            //for(int i = 0; i < 4; i++)
            //{
            //    msg[i] = byteVAng[i];
            //    msg[i + 4] = byteVLin[i];
            //}

            msg = Message.UartEncodeMessage((int)commandRobot.SetVitessePolaire, msg.Length, msg);
            serialPort.Write(msg, 0, msg.Length);
        }
        #endregion

        #region AncienRécupérageDeBalle
        //switch (stateRecupBall)
        //{
        //    case StateGoBalls.Waiting:
        //        SetInAutomaticMode();
        //        break;

        //    case StateGoBalls.State_Tout_Droit:
        //        vitesseGoBall[0] = 25; //vitesse gauche
        //        vitesseGoBall[1] = 25; //vitesse droite
        //        vitesseGoBall = Message.UartEncodeMessage((int)commandRobot.Set_PWM_Speed, vitesseGoBall.Length, vitesseGoBall);
        //        serialPort.Write(vitesseGoBall, 0, vitesseGoBall.Length);
        //        stateRecupBall = StateGoBalls.State_Tout_Droit_En_Cours;
        //        break;
        //    case StateGoBalls.State_Tout_Droit_En_Cours:
        //        SetInAutomaticMode();
        //        break;

        //    case StateGoBalls.State_Tourne_Droite_to_20:
        //        vitesseGoBall[0] = 20; //vitesse gauche
        //        vitesseGoBall[1] = 15; //vitesse droite
        //        vitesseGoBall = Message.UartEncodeMessage((int)commandRobot.Set_PWM_Speed, vitesseGoBall.Length, vitesseGoBall);
        //        serialPort.Write(vitesseGoBall, 0, vitesseGoBall.Length);
        //        stateRecupBall = StateGoBalls.State_Tourne_Droite_to_20_En_Cours;
        //        break;
        //    case StateGoBalls.State_Tourne_Droite_to_20_En_Cours:
        //        SetInAutomaticMode();
        //        break;

        //    case StateGoBalls.State_Tourne_Droite_to_50:
        //        vitesseGoBall[0] = 18; //vitesse gauche
        //        vitesseGoBall[1] = 10; //vitesse droite
        //        vitesseGoBall = Message.UartEncodeMessage((int)commandRobot.Set_PWM_Speed, vitesseGoBall.Length, vitesseGoBall);
        //        serialPort.Write(vitesseGoBall, 0, vitesseGoBall.Length);
        //        stateRecupBall = StateGoBalls.State_Tourne_Droite_to_50_En_Cours;
        //        break;
        //    case StateGoBalls.State_Tourne_Droite_to_50_En_Cours:
        //        SetInAutomaticMode();
        //        break;

        //    case StateGoBalls.State_Tourne_Droite:
        //        vitesseGoBall[0] = 18; //vitesse gauche
        //        vitesseGoBall[1] = 10; //vitesse droite
        //        vitesseGoBall = Message.UartEncodeMessage((int)commandRobot.Set_PWM_Speed, vitesseGoBall.Length, vitesseGoBall);
        //        serialPort.Write(vitesseGoBall, 0, vitesseGoBall.Length);
        //        stateRecupBall = StateGoBalls.State_Tourne_Droite_En_Cours;
        //        break;
        //    case StateGoBalls.State_Tourne_Droite_En_Cours:
        //        SetInAutomaticMode();
        //        break;

        //    case StateGoBalls.State_Tourne_Gauche_to_20:
        //        vitesseGoBall[0] = 15; //vitesse gauche
        //        vitesseGoBall[1] = 20; //vitesse droite
        //        vitesseGoBall = Message.UartEncodeMessage((int)commandRobot.Set_PWM_Speed, vitesseGoBall.Length, vitesseGoBall);
        //        serialPort.Write(vitesseGoBall, 0, vitesseGoBall.Length);
        //        stateRecupBall = StateGoBalls.State_Tourne_Gauche_to_20_En_Cours;
        //        break;
        //    case StateGoBalls.State_Tourne_Gauche_to_20_En_Cours:
        //        SetInAutomaticMode();
        //        break;

        //    case StateGoBalls.State_Tourne_Gauche_to_50:
        //        vitesseGoBall[0] = 10; //vitesse gauche
        //        vitesseGoBall[1] = 18; //vitesse droite
        //        vitesseGoBall = Message.UartEncodeMessage((int)commandRobot.Set_PWM_Speed, vitesseGoBall.Length, vitesseGoBall);
        //        serialPort.Write(vitesseGoBall, 0, vitesseGoBall.Length);
        //        stateRecupBall = StateGoBalls.State_Tourne_Gauche_to_50_En_Cours;
        //        break;
        //    case StateGoBalls.State_Tourne_Gauche_to_50_En_Cours:
        //        SetInAutomaticMode();
        //        break;

        //    case StateGoBalls.State_Tourne_Gauche:
        //        vitesseGoBall[0] = 10; //vitesse gauche
        //        vitesseGoBall[1] = 18; //vitesse droite
        //        vitesseGoBall = Message.UartEncodeMessage((int)commandRobot.Set_PWM_Speed, vitesseGoBall.Length, vitesseGoBall);
        //        serialPort.Write(vitesseGoBall, 0, vitesseGoBall.Length);
        //        stateRecupBall = StateGoBalls.State_Tourne_Gauche_En_Cours;
        //        break;
        //    case StateGoBalls.State_Tourne_Gauche_En_Cours:
        //        SetInAutomaticMode();
        //        break;

        //}



        //private void SetInAutomaticMode()
        //{
        //    StateGoBalls nextStateGoBalls = StateGoBalls.Waiting; 

        //    //TOUT DROIT
        //    if (angleBallePlusProche <= 0.08726646 && angleBallePlusProche >= (-0.08726646)) //de -5 à 5°
        //    {
        //        nextStateGoBalls = StateGoBalls.State_Tout_Droit;
        //    }
        //    //TOURNE GAUCHE
        //    else
        //    if (angleBallePlusProche > 0.08726646 && angleBallePlusProche <= 0.34906585) //de 5 à 20°
        //    {
        //        nextStateGoBalls = StateGoBalls.State_Tourne_Gauche_to_20;
        //    }
        //    else
        //    if (angleBallePlusProche > 0.34906585 && angleBallePlusProche <= 0.87266463) //de 20 à 50°
        //    {
        //        nextStateGoBalls = StateGoBalls.State_Tourne_Gauche_to_50;
        //    }
        //    else
        //    if (angleBallePlusProche > 0.87266463) // > à 50°
        //    {
        //        nextStateGoBalls = StateGoBalls.State_Tourne_Gauche;
        //    }
        //    //TOURNE DROITE

        //    else
        //    if (angleBallePlusProche < (-0.08726646) && angleBallePlusProche >= (-0.34906585)) //de -5 à -20°
        //    {
        //        nextStateGoBalls = StateGoBalls.State_Tourne_Droite_to_20;
        //    }
        //    else
        //    if (angleBallePlusProche < (-0.34906585) && angleBallePlusProche >= (-0.87266463)) //de -20 à -50°
        //    {
        //        nextStateGoBalls = StateGoBalls.State_Tourne_Droite_to_50;
        //    }
        //    else
        //    if (angleBallePlusProche < (-0.87266463)) // < à -50°
        //    {
        //        nextStateGoBalls = StateGoBalls.State_Tourne_Droite;
        //    }


        //    if(nextStateGoBalls != (stateRecupBall - 1))
        //    {
        //        stateRecupBall = nextStateGoBalls;
        //    }
        //}
        #endregion

        #region Timer
        private void TimerTraj_Tick(object sender, EventArgs e)
        {        
            robot.locationRefterrain = new PointD(robot.positionXOdo, robot.positionYOdo);
            var speed = trajectoryGenerator.GenerateSpeedConsigne(new PointD(robot.positionXOdo, robot.positionYOdo), robot.vitesseLineaire, robot.angle);
            UartSendVitesseConsigne(speed[0], speed[1]);
        }

        private void TimerOdo_Tick(object sender, EventArgs e)
        {
            lock (robot){
                //Ici on actualise les données odométriques
                if (robot.flagOdoIsUpdated == true)
                {
                    double ghostVX = trajectoryGenerator.ghostLocation.Vlin * Math.Cos(trajectoryGenerator.ghostLocation.Vtheta);
                    double ghostVY = trajectoryGenerator.ghostLocation.Vlin * Math.Sin(trajectoryGenerator.ghostLocation.Vtheta);

                    Location robotPosition = new Location(robot.positionXOdo, robot.positionYOdo, robot.angle, robot.vitesseLineaireX,
                                                            robot.vitesseLineaireY, robot.vitesseAngulaire);
                    Location ghostLocation = new Location(trajectoryGenerator.ghostLocation.X, trajectoryGenerator.ghostLocation.Y, trajectoryGenerator.ghostLocation.Theta, ghostVX,
                                                           ghostVY, trajectoryGenerator.ghostLocation.Vtheta);
                    Location wayPointLocation = new Location(trajectoryGenerator.wayPointLocation.X, trajectoryGenerator.wayPointLocation.Y, trajectoryGenerator.wayPointLocation.Theta, 0,
                                                           0, 0);

                    double consigneAngle=0;
                    worldMapDisplay.UpdateRobotLocation(robotPosition);
                    worldMapDisplay.UpdateRobotWaypoint(wayPointLocation);
                    worldMapDisplay.UpdateGhostLocation(ghostLocation);

                    if (trajectoryGenerator.wayPointLocation!=null)
                    consigneAngle = Math.Atan2(trajectoryGenerator.wayPointLocation.Y, trajectoryGenerator.wayPointLocation.X);
                    osciloPosition.AddPointToLine(0, robot.timestamp, robot.angle);
                    osciloPosition.AddPointToLine(1, robot.timestamp, consigneAngle); //consigneAngle

                    osciloVitesseLineaire.AddPointToLine(0, robot.timestamp, robot.consigneVitesseLineaire);
                    osciloVitesseLineaire.AddPointToLine(1, robot.timestamp, robot.vitesseLineaire);
                    osciloVitesseLineaire.AddPointToLine(2, robot.timestamp, trajectoryGenerator.ghostLocation.Vlin);

                    osciloVitesseAngulaire.AddPointToLine(0, robot.timestamp, robot.consigneVitesseAngulaire);
                    osciloVitesseAngulaire.AddPointToLine(1, robot.timestamp, robot.vitesseAngulaire);
                    osciloVitesseAngulaire.AddPointToLine(2, robot.timestamp, trajectoryGenerator.ghostLocation.Vtheta);


                    robot.flagOdoIsUpdated = false;
                }
            }
        
       }
     
        private void TimerLidar_Tick(object sender, EventArgs e)
        {
            lock (robot)
            {
                List<PointD> pointLidarCartesien = new List<PointD>();

                //Affichage à 10Hz des points Lidar
                if (robot.flagLidarUpdated == true)
                {
                    //grapheLidar.PointListClear("Lidar Data");

                    for (int i = 0; i < robot.FilteredLidarPointsXList.Count; i++)
                    {
                        pointLidarCartesien.Add(new PointD(robot.FilteredLidarPointsXList[i], robot.FilteredLidarPointsYList[i]));
                    }

                    robot.flagLidarUpdated = false;
                }

            
                //Affichage à 10Hz des balles sur le terrain
                if (robot.flagBallListUpdated == true)
                {

                    int nombrePointAffichBalle = 40;

                    for (int i = 0; i < robot.ballList.Count(); i++)
                    {
                        var distance = robot.ballList.Select(r => r.distance).ToList();
                        var angle = robot.ballList.Select(r => r.angle).ToList();

                        for (int j = 0; j < nombrePointAffichBalle; j++)
                        {
                            pointLidarCartesien.Add(new PointD(distance[i]*Math.Cos(angle[i]) + 0.1 * Math.Cos(2 * Math.PI * (double)j / nombrePointAffichBalle), 
                                                                                     distance[i]*Math.Sin(angle[i]) + 0.1 * Math.Sin(2 * Math.PI * (double)j  / nombrePointAffichBalle)));
                        }
                    }

                    worldMapDisplay.UpdateLidarMap(pointLidarCartesien);
                }

                robot.flagBallListUpdated = false;
            }
            
           


            if (auto == 0) {

                checkBoxAuto.IsChecked = false;

                #region Slideur Vitesse Manuelle
                //double vitesseManuelleDroite = slidebarVitesseDroit.Value;
                //double vitesseManuelleGauche = slidebarVitesseGauche.Value;

                //byte[] vitesseManuel = new byte[2];
                //vitesseManuel[0] = (byte)vitesseManuelleGauche;
                //vitesseManuel[1] = (byte)vitesseManuelleDroite;
                //vitesseManuel = Message.UartEncodeMessage((int)commandRobot.Set_PWM_Speed, vitesseManuel.Length, vitesseManuel);
                //serialPort.Write(vitesseManuel, 0, vitesseManuel.Length);
                #endregion

            } else
                checkBoxAuto.IsChecked = true;

            //Console.WriteLine("ss");
            //while (byteListReceived.Count>0)
            //{                
            //    byte c = byteListReceived.Dequeue();
            //    DecodeMessage(c);
            //   // textBoxReception.Text += "0x" + c.ToString("X2") + " ";
            //}          



        }

        Sequence carrePlusRecupBall = Sequence.coinBasDroite;
        int i = 0;
        private void TimerCarre_Tick(object sender, EventArgs e)
        {

            switch (carrePlusRecupBall)
            {
                case Sequence.coinBasDroite:
                    trajectoryGenerator.SetCurrentWayPoint(new PointD(-0.5, 0.5));

                    if (robot.locationRefterrain.X <= -0.45 && robot.locationRefterrain.Y >= 0.45)
                        carrePlusRecupBall = Sequence.coinBasGauche;

                    break;

                case Sequence.coinBasGauche:
                    trajectoryGenerator.SetCurrentWayPoint(new PointD(-0.5, -0.5));
                    if (robot.locationRefterrain.X <= -0.45 && robot.locationRefterrain.Y <= -0.45)
                        carrePlusRecupBall = Sequence.coinHautGauche;
                    break;

                case Sequence.coinHautGauche:
                    trajectoryGenerator.SetCurrentWayPoint(new PointD(0.5, -0.5));
                    if (robot.locationRefterrain.X >= 0.45 && robot.locationRefterrain.Y <= -0.45)
                        carrePlusRecupBall = Sequence.coinHautDroite;
                    break;

                case Sequence.coinHautDroite:
                    trajectoryGenerator.SetCurrentWayPoint(new PointD(0.5, 0.5));
                    if (robot.locationRefterrain.X >= 0.45 && robot.locationRefterrain.Y >= 0.45)
                        carrePlusRecupBall = Sequence.count;
                    break;

                case Sequence.count:
                    i++;
                    if (i > 2)
                        carrePlusRecupBall = Sequence.goBall;
                    else
                        carrePlusRecupBall = Sequence.coinBasDroite;
                    break;

                case Sequence.goBall:
                    //Ball_Detection();

                    break;

                default:
                    carrePlusRecupBall = Sequence.coinBasDroite;
                    break;

            }

        }

        #endregion

        #region DataReceivedSerialPort
        private void SerialPort_DataReceived(object sender, DataReceivedArgs e)
        {
            //string s = Encoding.UTF8.GetString(e.Data, 0, e.Data.Length);
            //receivedText += s;
            foreach (byte d in e.Data)
            {
                //byteListReceived.Enqueue(d);
                DecodeMessage(d);
            }
        }
        #endregion

        #region Enum
        public enum Led
        {
            Orange = 0,
            Blanche = 1,
            Bleu = 2
        }

        public enum commandRobot
        {
            TransmissionText = 0x0080,
            ReglageLed = 0x0020,
            DistTelemIR = 0x0030,
            ConsigneVitesse = 0x0040,
            RobotState = 0x0050,
            RobotAutoControl = 0x0052,
            Set_PWM_Speed = 0x0053,
            Jack = 0x0010,
            PositionData = 0x0061,
            ConsignePolaire = 0x0062,
            SetVitessePolaire = 0x0063
        }

        public enum StateRobot
        {
            STATE_ATTENTE = 0,
            STATE_ATTENTE_EN_COURS = 1,
            STATE_AVANCE = 2,
            STATE_AVANCE_EN_COURS = 3,
            STATE_TOURNE_GAUCHE = 4,
            STATE_TOURNE_GAUCHE_EN_COURS = 5,
            STATE_TOURNE_DROITE = 6,
            STATE_TOURNE_DROITE_EN_COURS = 7,
            STATE_TOURNE_SUR_PLACE_GAUCHE = 8,
            STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS = 9,
            STATE_TOURNE_SUR_PLACE_DROITE = 10,
            STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS = 11,
            STATE_ARRET = 12,
            STATE_ARRET_EN_COURS = 13,
            STATE_RECULE = 14,
            STATE_RECULE_EN_COURS = 15
        }

        public enum verif
        {
            correct,
            wrong
        }
        verif checksum = verif.wrong;


        public enum StateReception
        {
            Waiting,
            FunctionMSB,
            FunctionLSB,
            PayloadLengthMSB,
            PayloadLengthLSB,
            Payload,
            CheckSum
        }

        public enum Sequence
        {
            coinHautGauche,
            coinHautDroite,
            coinBasDroite,
            coinBasGauche,
            goBall,
            count
        }

        //public enum StateGoBalls
        //{
        //    State_Tout_Droit = 1,
        //    State_Tout_Droit_En_Cours = 2,
        //    State_Tourne_Droite_to_20 = 3,
        //    State_Tourne_Droite_to_20_En_Cours = 4,
        //    State_Tourne_Droite_to_50 = 5,
        //    State_Tourne_Droite_to_50_En_Cours = 6,
        //    State_Tourne_Droite = 7,
        //    State_Tourne_Droite_En_Cours = 8,
        //    State_Tourne_Gauche_to_20 = 9,
        //    State_Tourne_Gauche_to_20_En_Cours = 10,
        //    State_Tourne_Gauche_to_50 = 11,
        //    State_Tourne_Gauche_to_50_En_Cours = 12,
        //    State_Tourne_Gauche = 13,
        //    State_Tourne_Gauche_En_Cours = 14,
        //    Waiting = 15
        // }

        public enum StateToBalls
        {
            Waiting,
            FrontOf,
            Turn,
            Stop
        }


        #endregion

        #region InterfaceGraphique

        private void ButtonEnvoyer_Click(object sender, RoutedEventArgs e)
        {
            SendMessage();
        }
        private void Window_Closed(object sender, EventArgs e)
        {
            robotLidar.Stop();
        }
        private void OscilotStop_Click(object sender, RoutedEventArgs e)
        {
           if (OscilotStop.Content == "OscilotStop")
            {
                timerOdo.Stop();
                OscilotStop.Content = "OscilotStart";
            }
           else
            {
                timerOdo.Start();
                OscilotStop.Content = "OscilotStop";
            }
            
        }

        private void ButtonClear_Click(object sender, RoutedEventArgs e)
        {
            osciloPosition.ClearLine(0, "angle");
            osciloPosition.ClearLine(1, "consigne angle");

            osciloVitesseLineaire.ClearLine(0, "consigne vitesse angulaire");
            osciloVitesseLineaire.ClearLine(1, "vitesse angulaire");

            osciloVitesseAngulaire.ClearLine(0, "consigne vitesse Angulaire");
            osciloVitesseAngulaire.ClearLine(1, "vitesse Angulaire");

        }

        private void ButtonTest_Click(object sender, RoutedEventArgs e)
        {
            //string s = "Bonjour";
            //byte[] array = Encoding.ASCII.GetBytes(s);
            //var msg = Message.UartEncodeMessage(0x0080, s.Length, array);
            //serialPort.Write(msg, 0, msg.Length);

            //byte[] array2 = new byte[2];
            //array2[0] = 1;
            //array2[1] = 1;
            //array2 = Message.UartEncodeMessage((int)commandRobot.ReglageLed, array2.Length, array2);
            //serialPort.Write(array2, 0, array2.Length);


            //byte[] array3 = new byte[5];
            //array3[0] = 1;
            //array3[1] = 50;
            //array3[2] = 220;
            //array3[3] = 45;
            //array3[4] = 73;
            //array3 = Message.UartEncodeMessage((int)commandRobot.DistTelemIR, array3.Length, array3);
            //serialPort.Write(array3, 0, array3.Length);

            //byte[] array4 = new byte[2];
            //array4[0] = 50;
            //array4[1] = 10;
            //array4 = Message.UartEncodeMessage((int)commandRobot.ConsigneVitesse, array4.Length, array4);
            //serialPort.Write(array4, 0, array4.Length);
        }

        private void checkBoxAuto_Click(object sender, RoutedEventArgs e)
        {
            if (auto == 1)
            {
                auto = 0;
                
            }
            else if (auto == 0)
            {
                auto = 1;
                
            }



            byte[] messageAuto = new byte[2];
            messageAuto[0] = auto;
            messageAuto[1] = 127;
            messageAuto = Message.UartEncodeMessage((int)commandRobot.RobotAutoControl, messageAuto.Length, messageAuto);
            serialPort.Write(messageAuto, 0, messageAuto.Length);
        }


        private void Grid_KeyUp(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == Key.Enter)
            {
                SendMessage();
            }

            if (e.Key == Key.Delete)
            {
                textBoxReception.Clear();
            }

        }
        private void test0_0_Click(object sender, RoutedEventArgs e)
        {
            trajectoryGenerator.SetCurrentWayPoint(new PointD(0, 0));
        }

        private void Test01_Click(object sender, RoutedEventArgs e)
        {

            trajectoryGenerator.SetCurrentWayPoint(new PointD(0, 1));

        }

        private void Test10_Click(object sender, RoutedEventArgs e)
        {

            trajectoryGenerator.SetCurrentWayPoint(new PointD(0,-1));

        }

        private void Test1_Click(object sender, RoutedEventArgs e)
        {
            trajectoryGenerator.SetCurrentWayPoint(new PointD(1, 0));
        }
        private void test_10_Click(object sender, RoutedEventArgs e)
        {
            trajectoryGenerator.SetCurrentWayPoint(new PointD(-1, 0));
        }

        
        private void buttonCarre_Click(object sender, RoutedEventArgs e)
        {
            // TODO : on fait 3 fois un carré de 50x50cm et après on vas chercher une balle 
            //1er test avec un button après a programmer een fonctionnement normal ctrl +D copi la même ligne en dessous

            if (buttonCarre.Content == "StartSequence")
            {
                timerCarre.Start();
                buttonCarre.Content = "StopSequence";
            }
            else
            {
                timerCarre.Stop();
                buttonCarre.Content = "StartSequence";
            }


        }

        #endregion

        #region SendMessage
        //--------------------------------------SEND MESSAGE-----------------------------------------------
        private void SendMessage()
        {
            string s = textBoxEmission.Text;
            byte[] array = Encoding.ASCII.GetBytes(s);
            var msg = Message.UartEncodeMessage(0x0080, s.Length, array);
            serialPort.Write(msg, 0, msg.Length);
            //textBoxReception.Text += "Recu : " + textBoxEmission.Text + "\n";
            textBoxEmission.Clear();
        }
        #endregion

        #region DecodeMesasge
        StateReception rcvState = StateReception.Waiting;
        int msgDecodedFunction = 0;
        int msgDecodedPayloadLength = 0;
        byte[] msgDecodedPayload;
        int msgDecodedPayloadIndex;
        int bufferSize = 128;
        byte auto = 1;

        //-------------------------------------------------------------DECODE MESSAGE--------------------------------------
        Stopwatch sw = new Stopwatch();

        private void DecodeMessage(byte c)
        {
            switch(rcvState)
            {
                case StateReception.Waiting:
                    if (c == 0xFE)
                    {
                        //On initialise ce qui doit l'être
                        rcvState = StateReception.FunctionMSB;
                        msgDecodedFunction = 0;
                        msgDecodedPayloadLength = 0;
                    }
                    break;

                case StateReception.FunctionMSB:
                    msgDecodedFunction = c << 8;
                    rcvState = StateReception.FunctionLSB;
                    break;
                case StateReception.FunctionLSB:
                    msgDecodedFunction += c;
                    rcvState = StateReception.PayloadLengthMSB;
                    break;

                case StateReception.PayloadLengthMSB:
                    msgDecodedPayloadLength = c << 8;
                    rcvState = StateReception.PayloadLengthLSB;
                    break;
                case StateReception.PayloadLengthLSB:
                    msgDecodedPayloadLength += c;

                    msgDecodedPayload = new byte[msgDecodedPayloadLength];
                    msgDecodedPayloadIndex = 0;

                    if (msgDecodedPayloadLength == 0)
                        rcvState = StateReception.CheckSum;
                    else if(msgDecodedPayloadLength > 1024)
                        rcvState = StateReception.Waiting;
                    else
                    rcvState = StateReception.Payload;
                    break;

                case StateReception.Payload:
                    msgDecodedPayload[msgDecodedPayloadIndex++] = c;
                    if(msgDecodedPayloadIndex >= msgDecodedPayloadLength)
                        rcvState = StateReception.CheckSum;
                    break;

                case StateReception.CheckSum:

                    int checkSumReceived = c;
                    int checkSumDecoded = Message.CalculateChecksum(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                    if (checkSumReceived == checkSumDecoded)
                    {
                        //textBoxReception.Text += " Trame bonne\n" ;
                        checksum = verif.correct;

                        ProcessDecodedMessage(msgDecodedFunction, msgDecodedPayloadLength, msgDecodedPayload);
                    }
                    else
                    {
                        //textBoxReception.Text += "Error =  Calculate checksum :  " + checkSumDecoded + "  CheckSum received : " + checkSumReceived + "\n";
                        //checksum = verif.wrong;
                    }
                        rcvState = StateReception.Waiting;
                    break;

                default:
                    rcvState = StateReception.Waiting;
                    break;

            }
        }








        #endregion

        #region ProcessDecodeMssage
        //-------------------------------------------------PROCESS DECODE MESSAGE-----------------------------------------------


        void ProcessDecodedMessage(int msgFunction, int msgPayloadLength, byte[] msgPayload)
        {
            if (checksum == verif.correct)
            {
                switch (msgFunction)
                {
                    case (int)commandRobot.TransmissionText:
                        Dispatcher.BeginInvoke((Action)(() =>
                        {
                            string message = Encoding.ASCII.GetString(msgPayload);
                            textBoxReception.Text += "decode = " + message;
                        }));
                        break;
                    case (int)commandRobot.ReglageLed:
                        Dispatcher.BeginInvoke((Action)(() =>
                        {
                            if (msgPayload[0] == 0) //si orange
                            {
                                if (msgPayload[1] == 1)
                                    checkBoxLedOrange.IsChecked = true;
                                else
                                    checkBoxLedOrange.IsChecked = false;
                            }

                            if (msgPayload[0] == 1) //si blanche
                            {
                                if (msgPayload[1] == 1)
                                    checkBoxLedBlanche.IsChecked = true;
                                else
                                    checkBoxLedBlanche.IsChecked = false;
                            }

                            if (msgPayload[0] == 2) //si bleue
                            {
                                if (msgPayload[1] == 1)
                                    checkBoxLedBleu.IsChecked = true;
                                else
                                    checkBoxLedBleu.IsChecked = false;
                            }
                        }));
                        break;

                    case (int)commandRobot.DistTelemIR:

                        break;

                    case (int)commandRobot.ConsigneVitesse:
                        Dispatcher.BeginInvoke((Action)(() =>
                        {
                            LabelConsigneMotgauche.Content = msgPayload[0] * 0.025 + "m/s";
                            LabelConsigneMotdroit.Content = msgPayload[1] * 0.025 + "m/s";

                            robot.consigneMoteurGauche = msgPayload[0] * 0.025; // m/s
                            robot.consigneMoteurDroit = msgPayload[1] * 0.025; // m/s
                        }));
                        break;

                    case (int)commandRobot.Jack:
                        Dispatcher.BeginInvoke((Action)(() =>
                        {
                            if (msgPayload[0] == 1)
                            {
                                checkBoxJack.IsChecked = true;
                                jackState = 1;
                                //depart.Play();
                            }
                            else
                            {
                                checkBoxJack.IsChecked = false;
                                jackState = 0;
                                // depart.Stop();
                            }
                        }));
                        break;

                    case (int)commandRobot.RobotState:
                        Dispatcher.BeginInvoke((Action)(() =>
                        {
                            int instant = (((int)msgPayload[1]) << 24) + (((int)msgPayload[2]) << 16) + (((int)msgPayload[3]) << 8) + ((int)msgPayload[4]);
                            textBoxReception.Text += "\nRobot State : " + ((StateRobot)(msgPayload[0])).ToString() + " - " + instant.ToString() + " ms";
                        }));
                        break;
                    case (int)commandRobot.PositionData:

                        //robot.positionXOdo = -3.975;
                        //robot.positionYOdo = 0;
                        //robot.angle = 0;
                        //robot.vitesseLineaire = 0;
                        //robot.vitesseAngulaire = 0;

                        // pour recupérer les float du tab de byte entre 3 et 6 dans un tableau
                        //byte[] tab = msgPayload.GetRange(3,6); 
                        //tab.GetFloat();

                        byte[] tab = msgPayload.GetRange(0, 4); //On recupere 4 octetcs (de msgPayload) a partir de la position 0
                        robot.timestamp = (tab[0] << 24) | (tab[1] << 16) | (tab[2] << 8) | (tab[3] << 0);
                        tab = msgPayload.GetRange(4, 4);
                        robot.positionXOdo = tab.GetFloat();
                        tab = msgPayload.GetRange(8, 4);
                        robot.positionYOdo = tab.GetFloat();
                        tab = msgPayload.GetRange(12, 4);
                        robot.angle = tab.GetFloat();
                        tab = msgPayload.GetRange(16, 4);
                        robot.vitesseLineaire = tab.GetFloat();
                        tab = msgPayload.GetRange(20, 4);
                        robot.vitesseAngulaire = tab.GetFloat();

                        robot.vitesseLineaireX = robot.vitesseLineaire * Math.Cos(robot.vitesseAngulaire);
                        robot.vitesseLineaireY = robot.vitesseLineaire * Math.Sin(robot.vitesseAngulaire);


                        Dispatcher.BeginInvoke((Action)(() =>
                        {
                            labelValpositionX.Content = Math.Round(robot.positionXOdo, 3) + " m";
                            labelValpositionY.Content = Math.Round(robot.positionYOdo, 3) + " m";
                            labelValAngle.Content = Math.Round(Toolbox.RadToDeg(robot.angle), 3) + " °";
                            labelValvitesseLineaire.Content = Math.Round(robot.vitesseLineaire, 3) + " m/s";
                            labelValvitesseAngulaire.Content = Math.Round(robot.vitesseAngulaire, 3) + " rad/s";
                        }));

                        Console.WriteLine("Temps entre deux arrivees de retour codeur : " + sw.ElapsedMilliseconds.ToString());
                        sw.Restart();
                        //var speed = trajectoryGenerator.GenerateSpeedConsigne(new PointD(robot.positionXOdo, robot.positionYOdo), robot.angle);
                        //UartSendVitesseConsigne(speed[0], speed[1]);

                        break;

                    case (int)commandRobot.ConsignePolaire:

                        robot.consigneVitesseLineaire = 0;
                        robot.consigneVitesseAngulaire = 0;

                        // pour recupérer les float du tab de byte entre 3 et 6 dans un tableau
                        //byte[] tab = msgPayload.GetRange(3,6); 
                        //tab.GetFloat();

                        byte[] tabPolaire = msgPayload.GetRange(0, 4); //On recupere 4 octetcs (de msgPayload) a partir de la position 0
                        robot.consigneVitesseAngulaire += tabPolaire.GetFloat();
                        tabPolaire = msgPayload.GetRange(4, 4);
                        robot.consigneVitesseLineaire += tabPolaire.GetFloat();

                        Dispatcher.BeginInvoke((Action)(() =>
                        {
                            labelConsignevitesseAngulaireVal.Content = Math.Round(robot.consigneVitesseAngulaire, 4) + " rad/s";
                            labelConsignevitesseLineaireVal.Content = Math.Round(robot.consigneVitesseLineaire, 4) + " m/s";
                        }));

                        robot.flagOdoIsUpdated = true;

                        break;
                }

            }
            //if (msgFunction == (int)pilotRobotFunctions.ReglageLed)
            //{
            //    if (msgPayload[1] == 0)//Orange
            //        if (msgPayload[0] == 1)
            //            checkBoxLedOrange.IsChecked = true;
            //        else
            //            checkBoxLedOrange.IsChecked = false;
            //    else if (msgPayload[1] == 1)//Blanche
            //        checkBoxLedBlanche.IsChecked = true;
            //    else
            //        checkBoxLedBlanche.IsChecked = false;

            //    else if (msgPayload[1] == 2)//Bleu
            //        checkBoxLedBleu.IsChecked = true;
            //    else
            //        checkBoxLedBleu.IsChecked = false;
            //}
            //else if (msgFunction == (int)pilotRobotFunctions.DistTelemIR)
            //{

            //}

        }
        #endregion
    }
}