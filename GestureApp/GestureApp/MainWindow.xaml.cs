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

// added libraries
using Microsoft.Kinect;
using LightBuzz.Vitruvius;
using uPLibrary.Networking.M2Mqtt;
using uPLibrary.Networking.M2Mqtt.Messages;

namespace GestureApp
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor _sensor;
        MultiSourceFrameReader _reader;
        GestureController _gestureController;

        bool mqttEnabled;
        static string ipAddress = "128.32.44.126";
        static MqttClient _mqttClient;
        byte _mqttCode;

        public MainWindow()
        {
            InitializeComponent();

            _sensor = KinectSensor.GetDefault();

            mqttEnabled = true;

            if (mqttEnabled == true)
            {
                _mqttClient = new MqttClient(ipAddress);
                _mqttCode = _mqttClient.Connect(Guid.NewGuid().ToString());
            }

            if (_sensor != null)
            {
                _sensor.Open();

                _reader = _sensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth 
                    | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
                _reader.MultiSourceFrameArrived += Reader_MultiSourceFrameArrived;

                _gestureController = new GestureController();
                _gestureController.GestureRecognized += GestureController_GestureRecognized;
            }
        }

        // read the frames coming in from Kinect, get the body from the frames
        void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            var reference = e.FrameReference.AcquireFrame();

            // RGB camera feed
            using (var frame = reference.ColorFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    if (viewer.Visualization == Visualization.Color)
                    {
                        viewer.Image = frame.ToBitmap();
                    }
                }
            }

            // Body tracking
            using (var frame = reference.BodyFrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    Body body = frame.Bodies().Closest(); // detect gesture of closest body

                    if (body != null)
                    {
                        _gestureController.Update(body);
                    }
                }
            }
        }

        // triggered when a gesture is recognized
        void GestureController_GestureRecognized(object sender, GestureEventArgs e)
        {
            GestureTextOutput.Content = e.GestureType.ToString();

            // mqtt publish
            if (mqttEnabled == true)
            {
                ushort msgId = _mqttClient.Publish("gesture", //topic
                                Encoding.UTF8.GetBytes(e.GestureType.ToString()), // message body
                                MqttMsgBase.QOS_LEVEL_EXACTLY_ONCE, //QoS level
                                false); // retained
                Console.WriteLine(msgId); // debug purposes
            }
        }

        private void KinectViewer_Loaded(object sender, RoutedEventArgs e)
        {
            Console.WriteLine("KinectViewer loaded!");
        }
    }
}
