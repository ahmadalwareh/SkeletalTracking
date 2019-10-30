// (c) Copyright Microsoft Corporation.
// This source is subject to the Microsoft Public License (Ms-PL).
// Please see http://go.microsoft.com/fwlink/?LinkID=131993 for details.
// All other rights reserved.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using Microsoft.Kinect;
using Coding4Fun.Kinect.Wpf;
using System.IO.Ports;
using System.Threading;
using System.Windows.Threading;

namespace SkeletalTracking
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
        }
        bool closing = false;
        const int skeletonCount = 6;
        Skeleton[] allSkeletons = new Skeleton[skeletonCount];
        private SerialPort NXTSERIALPORT;
        //private Boolean running;
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            System.Windows.Threading.DispatcherTimer dispatcherTimer = new System.Windows.Threading.DispatcherTimer();
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 1);
            dispatcherTimer.Start();
            kinectSensorChooser1.KinectSensorChanged += new DependencyPropertyChangedEventHandler(kinectSensorChooser1_KinectSensorChanged);
        }
        void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            check_ports();
        }
        void kinectSensorChooser1_KinectSensorChanged(object sender, DependencyPropertyChangedEventArgs e)
        {
            KinectSensor old = (KinectSensor)e.OldValue;
            StopKinect(old);
            KinectSensor sensor = (KinectSensor)e.NewValue;
            if (sensor == null)
            {
                return;
            }
            var parameters = new TransformSmoothParameters
            {
                Smoothing = 0.3f,
                Correction = 0.0f,
                Prediction = 0.0f,
                JitterRadius = 1.0f,
                MaxDeviationRadius = 0.5f
            };
            // sensor.SkeletonStream.Enable(parameters);
            sensor.SkeletonStream.Enable();
            sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(sensor_AllFramesReady);
            sensor.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            try
            {
                sensor.Start();
            }
            catch (System.IO.IOException)
            {
                kinectSensorChooser1.AppConflictOccurred();
            }
        }
        void sensor_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            if (closing)
            {
                return;
            }
            //Get a skeleton
            Skeleton first = GetFirstSkeleton(e);

            if (first == null)
            {
                return;
            }
            GetCameraPoint(first, e);
        }
        void GetCameraPoint(Skeleton first, AllFramesReadyEventArgs e)
        {
            using (DepthImageFrame depth = e.OpenDepthImageFrame())
            {
                if (depth == null ||
                    kinectSensorChooser1.Kinect == null)
                {
                    return;
                }
                DepthImagePoint lefthandpoint =
                   depth.MapFromSkeletonPoint(first.Joints[JointType.HandLeft].Position);
                DepthImagePoint righthandpoint =
                    depth.MapFromSkeletonPoint(first.Joints[JointType.HandRight].Position);
                DepthImagePoint hipcenter =
                   depth.MapFromSkeletonPoint(first.Joints[JointType.HipCenter].Position);
                DepthImagePoint shouldercent =
                  depth.MapFromSkeletonPoint(first.Joints[JointType.ShoulderCenter].Position);
                DepthImagePoint hipcent =
                  depth.MapFromSkeletonPoint(first.Joints[JointType.HipCenter].Position);
                DepthImagePoint head =
                  depth.MapFromSkeletonPoint(first.Joints[JointType.Head].Position);
                DepthImagePoint foot =
                 depth.MapFromSkeletonPoint(first.Joints[JointType.FootLeft].Position);
                DepthImagePoint foot1 =
                depth.MapFromSkeletonPoint(first.Joints[JointType.FootRight].Position);
                DepthImagePoint left_knee =
                depth.MapFromSkeletonPoint(first.Joints[JointType.KneeLeft].Position);
                Double leftdis = Math.Sqrt(Math.Pow(Math.Abs(lefthandpoint.Depth - hipcenter.Depth), 2));
                Double rightdis = Math.Sqrt(Math.Pow(Math.Abs(righthandpoint.Depth - hipcenter.Depth), 2));
                Double foot_knee_dist = foot1.Y - left_knee.Y;
                Int32 x = Convert.ToInt32(rightdis - leftdis);
                String result = "stop";
                if (foot_knee_dist < 10)
                {
                    if (NXTSERIALPORT != null)
                    {
                        if (NXTSERIALPORT.IsOpen == true)
                            send_pre(4, 0);
                    }
                    t1.Text = "backwords";
                }
                else
                {
                    if (x > 200)
                    {
                        result = "right motor";
                    }
                    if (x < -200)
                    {
                        result = "left motor";
                    }
                    else
                    {
                        if ((rightdis > 230) && (leftdis > 230))
                        {
                            result = "forword";
                        }
                        if ((rightdis < 80) && (leftdis < 80))
                        {
                            result = "stop";
                        }
                    }
                    t1.Text = result;
                    Int32 num = 0;
                    if (result == "stop")
                    {
                        num = 5;
                    }
                    if (result == "forword")
                    {
                        num = 1;
                    }
                    if (result == "left motor")
                    {
                        num = 3;
                    }
                    if (result == "right motor")
                    {
                        num = 2;
                    }
                    Double constdist = Math.Sqrt(Math.Pow(Math.Abs(shouldercent.Y - hipcent.Y), 2));
                    Double vardist = Math.Sqrt(Math.Pow(Math.Abs(head.Y - foot.Y), 2));
                    Double rate = (vardist / constdist);
                    if (rate < 2)
                    {
                        rate = 2;
                    }
                    if (rate > 4)
                    {
                        rate = 4;
                    }
                    rate = (rate - 2) * 5;
                    Int32 speed_rate;
                    speed_rate = 1;

                    speed_rate = 2;
                    if (NXTSERIALPORT != null)
                    {
                        if (NXTSERIALPORT.IsOpen == true)
                        {
                            send_pre(num, 0);
                        }
                    }
                }
            }
        }
        Skeleton GetFirstSkeleton(AllFramesReadyEventArgs e)
        {
            using (SkeletonFrame skeletonFrameData = e.OpenSkeletonFrame())
            {
                if (skeletonFrameData == null)
                {
                    return null;
                }
                skeletonFrameData.CopySkeletonDataTo(allSkeletons);
                //get the first tracked skeleton
                Skeleton first = (from s in allSkeletons
                                  where s.TrackingState == SkeletonTrackingState.Tracked
                                  select s).FirstOrDefault();
                return first;
            }
        }
        private void StopKinect(KinectSensor sensor)
        {
            if (sensor != null)
            {
                if (sensor.IsRunning)
                {
                    //stop sensor 
                    sensor.Stop();
                    //stop audio if not null
                    if (sensor.AudioSource != null)
                    {
                        sensor.AudioSource.Stop();
                    }
                }
            }
        }
        private void CameraPosition(FrameworkElement element, ColorImagePoint point)
        {
            Canvas.SetLeft(element, point.X - element.Width / 2);
            Canvas.SetTop(element, point.Y - element.Height / 2);
        }
        private void ScalePosition(FrameworkElement element, Joint joint)
        {
            //convert the value to X/Y
            Joint scaledJoint = joint.ScaleTo(1280, 720, .3f, .3f);
            Canvas.SetLeft(element, scaledJoint.Position.X);
            Canvas.SetTop(element, scaledJoint.Position.Y);
        }
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            closing = true;
            StopKinect(kinectSensorChooser1.Kinect);
            if (NXTSERIALPORT != null)
            {
                if (NXTSERIALPORT.IsOpen == true)
                {
                    try
                    {
                        send_pre(1, 0);
                    }
                    catch (System.IO.IOException)
                    { }
                }
            }
            try
            {
                if (NXTSERIALPORT != null)
                {
                    NXTSERIALPORT.Close();
                    NXTSERIALPORT.Dispose();
                }
            }
            catch (System.IO.IOException)
            { }
        }
        private void kinectColorViewer1_Loaded(object sender, RoutedEventArgs e)
        { }
        private void b1_Click(object sender, RoutedEventArgs e)
        {
            //  comboBox1.ItemsSource = SerialPort.GetPortNames();
        }
        private void b2_Click(object sender, RoutedEventArgs e)//connect
        {
            try
            {
                NXTSERIALPORT = new SerialPort((string)comboBox1.SelectedValue);
                NXTSERIALPORT.Close();
                NXTSERIALPORT.Open();
            }
            catch (System.IO.IOException) { MessageBox.Show("couldn't connect to robot"); }
        }
        private void check_ports()
        {
            try
            {
                if (NXTSERIALPORT != null)
                {
                    if (NXTSERIALPORT.IsOpen == true)
                    {
                        if (l2.Dispatcher.CheckAccess())
                        {
                            // The calling thread owns the dispatcher, and hence the UI element
                            l2.Content = "robot is connected ..";
                        }
                        else
                        {
                            Application.Current.Dispatcher.BeginInvoke(
                            DispatcherPriority.Background,
                            new Action(() => this.l2.Content = "robot is connected .."));
                        }
                    }
                    if (NXTSERIALPORT.IsOpen == false)
                    {
                        if (l2.Dispatcher.CheckAccess())
                        {
                            // The calling thread owns the dispatcher, and hence the UI element
                            l2.Content = "robot is disconnected ..";
                        }
                        else
                        {
                            Application.Current.Dispatcher.BeginInvoke(
                            DispatcherPriority.Background,
                            new Action(() => this.l2.Content = "robot is disconnected .."));
                        }
                    }
                }
                else
                {
                    if (l2.Dispatcher.CheckAccess())
                    {
                        // The calling thread owns the dispatcher, and hence the UI element
                        l2.Content = "robot is disconnected ..";
                    }
                    else
                    {
                        Application.Current.Dispatcher.BeginInvoke(
                        DispatcherPriority.Background,
                        new Action(() => this.l2.Content = "robot is disconnected .."));
                    }
                }
            }
            catch (System.NullReferenceException)
            { }
        }
        private void b3_Click(object sender, RoutedEventArgs e)//disconnect
        {
            try
            {
                NXTSERIALPORT.Close();
            }
            catch (System.IO.IOException)
            { }
        }
        private void send_message(string message, byte mailbox)
        {
            try
            {
                NXTSERIALPORT.Write(message);
            }
            catch (System.IO.IOException)
            {
                MessageBox.Show("can't send the message");
            }
        }
        private void send_pre(int data, byte mailbox)
        {
            if (data > 0)
            {
                try
                {
                    this.send_message(data.ToString(), mailbox);
                }
                catch (System.IO.IOException)
                {
                    MessageBox.Show("IO Error");
                }
            }
        }
        private void t1_TextChanged(object sender, TextChangedEventArgs e)
        { }
    }
}
