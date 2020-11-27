//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using OpenCvSharp;
using MathNet.Numerics;
using System.Windows.Threading;
using FaceBasics_WPFML.Model;
using Point = System.Windows.Point;

namespace Microsoft.Samples.Kinect.FaceBasics
{
    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : System.Windows.Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Thickness of face bounding box and face points
        /// </summary>
        private const double DrawFaceShapeThickness = 8;

        /// <summary>
        /// Font size of face property text 
        /// </summary>
        private const double DrawTextFontSize = 30;

        /// <summary>
        /// Radius of face point circle
        /// </summary>
        private const double FacePointRadius = 1.0;

        /// <summary>
        /// Text layout offset in X axis
        /// </summary>
        private const float TextLayoutOffsetX = -0.1f;

        /// <summary>
        /// Text layout offset in Y axis
        /// </summary>
        private const float TextLayoutOffsetY = -0.15f;

        /// <summary>
        /// Face rotation display angle increment in degrees
        /// </summary>
        private const double FaceRotationIncrementInDegrees = 5.0;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Formatted text to indicate that there are no bodies/faces tracked in the FOV
        /// </summary>
        [Obsolete]
        private FormattedText textFaceNotTracked = new FormattedText(
                        "No bodies or faces are tracked ...",
                        CultureInfo.GetCultureInfo("en-us"),
                        FlowDirection.LeftToRight,
                        new Typeface("Georgia"),
                        DrawTextFontSize,
                        Brushes.White);

        /// <summary>
        /// Text layout for the no face tracked message
        /// </summary>
        private System.Windows.Point textLayoutFaceNotTracked = new System.Windows.Point(10.0, 10.0);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array to store bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// Number of bodies tracked
        /// </summary>
        private int bodyCount;

        /// <summary>
        /// Face frame sources
        /// </summary>
        private FaceFrameSource[] faceFrameSources = null;

        /// <summary>
        /// Face frame readers
        /// </summary>
        private FaceFrameReader[] faceFrameReaders = null;

        /// <summary>
        /// Storage for face frame results
        /// </summary>
        private FaceFrameResult[] faceFrameResults = null;

        /// <summary>
        /// Width of display (color space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (color space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// Display rectangle
        /// </summary>
        private System.Windows.Rect displayRect;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// List of brushes for each face tracked
        /// </summary>
        private List<Brush> faceBrush;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// the reader for the the multiframes
        /// </summary>
        private MultiSourceFrameReader multiFrameSourceReader;

        /// <summary>
        /// The types of objects that can be tracked
        /// </summary>
        public enum TrackingType { Circle, Face, All, None };

        /// <summary>
        /// the color frame bitmap
        /// </summary>
        private WriteableBitmap bitmap = null;

        /// <summary>
        /// the size of the data that the buffer represents
        /// </summary>
        private uint bitmapBackBufferSize;

        /// <summary>
        /// current type of object beign tracked
        /// </summary>
        private TrackingType tracking;

        /// <summary>
        /// the number of frames that has been captured
        /// </summary>
        private int frameCounter;
        private int prevFrameCounter;
        private int FramesPerSecond = 0;

        /// <summary>
        /// an array of the current circles on screen
        /// </summary>
        private CircleSegment[] opencvCirclesHolder = new CircleSegment[0];

        /// <summary>
        /// the grey scale image of the color frame
        /// </summary>
        private Mat opencv8grey = new Mat();

        /// <summary>
        /// the inverse resolution of the grey scale image
        /// </summary>
        protected int dp_resolution;

        /// <summary>
        /// the number of pixels between a circle
        /// </summary>
        protected int minDistanceFromOtherCenter;

        /// <summary>
        /// the upper threshold of the canny circle detection
        /// </summary>
        protected int canny_upper_threshold;

        /// <summary>
        /// the confidence that a circle is a circle
        /// </summary>
        protected double confidence;

        /// <summary>
        /// the smallest a circle can be
        /// </summary>
        protected int minRadius;

        /// <summary>
        /// the largest a circle can be
        /// </summary>
        public int maxRadius;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;



        /// <summary>
        /// the bytes per pixel of a BGR 32 bit image
        /// </summary>
        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        private List<LinkedList<PointF>> faceLocationBuffer = new List<LinkedList<PointF>>();

        private List<LinkedList<PointF>> circleLocationBuffer = new List<LinkedList<PointF>>();

        private DispatcherTimer dispatcherTimer = new DispatcherTimer();
        private Stopwatch stopwatch = new Stopwatch();
        private int depthWidth;
        private int depthHeight;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// what else did you expect... its the main
        /// </summary>
        public MainWindow()
        {
            
            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;
            FrameDescription depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;
            this.depthWidth = depthFrameDescription.Width;
            this.depthHeight = depthFrameDescription.Height;
            // get the color frame details
            FrameDescription frameDescription = this.kinectSensor.ColorFrameSource.FrameDescription;

            // set the display specifics
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;
            this.displayRect = new System.Windows.Rect(0.0, 0.0, this.displayWidth, this.displayHeight);

            this.bitmap = new WriteableBitmap(this.displayWidth, this.displayHeight, 96.0, 96.0, PixelFormats.Bgra32, null);

            // Calculate the WriteableBitmap back buffer size
            this.bitmapBackBufferSize = (uint)((this.bitmap.BackBufferStride * (this.bitmap.PixelHeight - 1)) + (this.bitmap.PixelWidth * this.bytesPerPixel));

            // open the reader for the body frames
            //this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // wire handler for body frame arrival
            //this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;

            initBones();

            this.multiFrameSourceReader = this.kinectSensor.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Body);

            this.multiFrameSourceReader.MultiSourceFrameArrived += this.Reader_MultiSourceFrameArrived;

            stopwatch.Start();

            // set the maximum number of bodies that would be tracked by Kinect
            this.bodyCount = this.kinectSensor.BodyFrameSource.BodyCount;

            // allocate storage to store body objects
            this.bodies = new Body[this.bodyCount];

            // specify the required face frame results
            FaceFrameFeatures faceFrameFeatures =
                FaceFrameFeatures.BoundingBoxInColorSpace
                | FaceFrameFeatures.PointsInColorSpace
                | FaceFrameFeatures.RotationOrientation
                | FaceFrameFeatures.FaceEngagement
                | FaceFrameFeatures.Glasses
                | FaceFrameFeatures.Happy
                | FaceFrameFeatures.LeftEyeClosed
                | FaceFrameFeatures.RightEyeClosed
                | FaceFrameFeatures.LookingAway
                | FaceFrameFeatures.MouthMoved
                | FaceFrameFeatures.MouthOpen;

            // create a face frame source + reader to track each face in the FOV
            this.faceFrameSources = new FaceFrameSource[this.bodyCount];
            this.faceFrameReaders = new FaceFrameReader[this.bodyCount];
            for (int i = 0; i < this.bodyCount; i++)
            {
                // create the face frame source with the required face frame features and an initial tracking Id of 0
                this.faceFrameSources[i] = new FaceFrameSource(this.kinectSensor, 0, faceFrameFeatures);

                // open the corresponding reader
                this.faceFrameReaders[i] = this.faceFrameSources[i].OpenReader();

                //sets the face buffers to an empty linkedlist
                faceLocationBuffer.Add(new LinkedList<PointF>());
            }

            // allocate storage to store face frame results for each face in the FOV
            this.faceFrameResults = new FaceFrameResult[this.bodyCount];

            // populate face result colors - one for each face index
            this.faceBrush = new List<Brush>()
            {
                Brushes.DarkGray,
                Brushes.Red,
                Brushes.Blue,
                Brushes.MediumPurple,
                Brushes.Teal,
                Brushes.Black
            };

            this.bodyColors = new List<Pen>();
            this.bodyColors.Add(new Pen(Brushes.Red, 6));
            this.bodyColors.Add(new Pen(Brushes.Orange, 6));
            this.bodyColors.Add(new Pen(Brushes.Green, 6));
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6));
            this.bodyColors.Add(new Pen(Brushes.Violet, 6));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            tracking = TrackingType.Circle;

            this.dp_resolution = 8; //16                                        //resolution of the image to be processed inverse ratio 
            this.minDistanceFromOtherCenter = 50; //30                           //Minimum distance that a detected circle must be between an other circle
            this.canny_upper_threshold = 100; //100                              //detection threshold. The higher the faster the fps
            this.confidence = 90.0 / 100.0; //0.85                              //the percent confidence that a circle really is a circle
            this.minRadius = 50; //20                                            //minimum size that a circle must be greater than
            this.maxRadius = 250; // 1/3 the screen  //maximum size that a circle must be less than

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            //timer init
            dispatcherTimer.Tick += new EventHandler(dispatcherTimer_Tick);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 1);
            dispatcherTimer.Start();

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        private void initBones()
        {
            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;
        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }
        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }
        /// <summary>
        /// Converts rotation quaternion to Euler angles 
        /// And then maps them to a specified range of values to control the refresh rate
        /// </summary>
        /// <param name="rotQuaternion">face rotation quaternion</param>
        /// <param name="pitch">rotation about the X-axis</param>
        /// <param name="yaw">rotation about the Y-axis</param>
        /// <param name="roll">rotation about the Z-axis</param>
        private static void ExtractFaceRotationInDegrees(Vector4 rotQuaternion, out int pitch, out int yaw, out int roll)
        {
            double x = rotQuaternion.X;
            double y = rotQuaternion.Y;
            double z = rotQuaternion.Z;
            double w = rotQuaternion.W;

            // convert face rotation quaternion to Euler angles in degrees
            double yawD, pitchD, rollD;
            pitchD = Math.Atan2(2 * ((y * z) + (w * x)), (w * w) - (x * x) - (y * y) + (z * z)) / Math.PI * 180.0;
            yawD = Math.Asin(2 * ((w * y) - (x * z))) / Math.PI * 180.0;
            rollD = Math.Atan2(2 * ((x * y) + (w * z)), (w * w) + (x * x) - (y * y) - (z * z)) / Math.PI * 180.0;

            // clamp the values to a multiple of the specified increment to control the refresh rate
            double increment = FaceRotationIncrementInDegrees;
            pitch = (int)(Math.Floor((pitchD + ((increment / 2.0) * (pitchD > 0 ? 1.0 : -1.0))) / increment) * increment);
            yaw = (int)(Math.Floor((yawD + ((increment / 2.0) * (yawD > 0 ? 1.0 : -1.0))) / increment) * increment);
            roll = (int)(Math.Floor((rollD + ((increment / 2.0) * (rollD > 0 ? 1.0 : -1.0))) / increment) * increment);
        }
        /// <summary>
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            for (int i = 0; i < this.bodyCount; i++)
            {
                if (this.faceFrameReaders[i] != null)
                {
                    // wire handler for face frame arrival
                    this.faceFrameReaders[i].FrameArrived += this.Reader_FaceFrameArrived;
                }
            }

            if (this.bodyFrameReader != null)
            {
                // wire handler for body frame arrival
                this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;
            }
        }
        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            for (int i = 0; i < this.bodyCount; i++)
            {
                if (this.faceFrameReaders[i] != null)
                {
                    // FaceFrameReader is IDisposable
                    this.faceFrameReaders[i].Dispose();
                    this.faceFrameReaders[i] = null;
                }

                if (this.faceFrameSources[i] != null)
                {
                    // FaceFrameSource is IDisposable
                    this.faceFrameSources[i].Dispose();
                    this.faceFrameSources[i] = null;
                }
            }

            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }
        /// <summary>
        /// Handles the face frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FaceFrameArrived(object sender, FaceFrameArrivedEventArgs e)
        {
            using (FaceFrame faceFrame = e.FrameReference.AcquireFrame())
            {
                if (faceFrame != null)
                {
                    // get the index of the face source from the face source array
                    int index = this.GetFaceSourceIndex(faceFrame.FaceFrameSource);

                    // check if this face frame has valid face frame results
                    if (this.ValidateFaceBoxAndPoints(faceFrame.FaceFrameResult))
                    {
                        // store this face frame result to draw later
                        this.faceFrameResults[index] = faceFrame.FaceFrameResult;
                    }
                    else
                    {
                        // indicates that the latest face frame result from this reader is invalid
                        this.faceFrameResults[index] = null;
                    }
                }
            }
        }
        /// <summary>
        /// Returns the index of the face frame source
        /// </summary>
        /// <param name="faceFrameSource">the face frame source</param>
        /// <returns>the index of the face source in the face source array</returns>
        private int GetFaceSourceIndex(FaceFrameSource faceFrameSource)
        {
            int index = -1;

            for (int i = 0; i < this.bodyCount; i++)
            {
                if (this.faceFrameSources[i] == faceFrameSource)
                {
                    index = i;
                    break;
                }
            }

            return index;
        }
        /// <summary>
        /// takes the given frames and finds/tracks the set tracking object and sets the frame to be drawen
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>

        private void drawCurve(LinkedList<PointF> buffer, DrawingContext dc, int PolyOrder)
        {
            double[] facePosX = new double[buffer.Count];
            double[] facePosY = new double[buffer.Count];
            int bufferIdx = 0;
            foreach (PointF p in buffer)
            {
                facePosX[bufferIdx] = p.X;
                facePosY[bufferIdx] = p.Y;
                bufferIdx++;
            }
            Point prevPoint = new Point();
            for (int i = 0; i < facePosX.Length; i++)
            {
                Point currentPoint = new Point(facePosX[i], facePosY[i]);
                if (i != 0)
                    dc.DrawLine(new Pen(Brushes.Green, 10), prevPoint, currentPoint);
                prevPoint = currentPoint;
            }

            double[] polyFitCurve = Fit.Polynomial(facePosX, facePosY, PolyOrder);
            
            for (int j = 0; j < facePosX.Length; j++)
            {
                double x = facePosX[j];
                double y = 0;
                for (int k = PolyOrder; k >= 0; k--)
                {
                    y += polyFitCurve[k] * Math.Pow(x, k);
                }

                System.Windows.Point p = new System.Windows.Point(x, y);
                if (j != 0)
                    dc.DrawLine(new Pen(Brushes.Blue, 10), prevPoint, p);
                prevPoint = p;
            }

            for (int j = 0; j < facePosX.Length; j++)
            {
                double xScale = facePosX[facePosX.Length - 2] - facePosX[facePosX.Length - 1];
                double x = facePosX[0] + xScale * j;
                double y = 0;
                for (int k = PolyOrder; k >= 0; k--)
                {
                    y += polyFitCurve[k] * Math.Pow(x, k);
                }

                System.Windows.Point p = new System.Windows.Point(x, y);
                if (j != 0)
                    dc.DrawLine(new Pen(Brushes.Red, 10), prevPoint, p);
                prevPoint = p;
            }
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        private void Reader_MultiSourceFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            ColorFrame colorFrame = null;
            BodyFrame bodyFrame = null;
            bool isBitmapLocked = false;


            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            // If the Frame has expired by the time we process this event, return.
            if (multiSourceFrame == null)
            {
                return;
            }

            // We use a try/finally to ensure that we clean up before we exit the function.  
            // This includes calling Dispose on any Frame objects that we may have and unlocking the bitmap back buffer.
            try
            {
                colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame();
                bitmap.Lock();
                isBitmapLocked = true;
                //Process color frame

                bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame();
                if (colorFrame != null)
                    colorFrame.CopyConvertedFrameDataToIntPtr(bitmap.BackBuffer, bitmapBackBufferSize, ColorImageFormat.Bgra);
                // Process body and face data
                if (bodyFrame != null)
                {
                    using (DrawingContext dc = drawingGroup.Open())
                    {
                        //WriteableBitmap _wb = new WriteableBitmap(Convert(opencv8grey.Canny(this.drawingCanny1, this.drawingCanny2).ToBitmap()));
                        //_wb.WritePixels(new Int32Rect(0, 0, 1920, 1080), data, _stride,0);
                        //_wb.WritePixels(new Int32Rect(0, 0, 1920, 1080), opencv8grey.CvPtr, bufferSize, _stride);
                        //draw the color frame
                        dc.DrawImage(bitmap, displayRect);
                        bool drawFaceResult = false;
                        if (tracking == TrackingType.Face || tracking == TrackingType.All)
                        {
                            //Fit.Polynomial()
                            bodyFrame.GetAndRefreshBodyData(bodies);
                            // iterate through each face source
                            for (int i = 0; i < this.bodyCount; i++)
                            {
                                Pen drawPen = this.bodyColors[i];
                                // check if a valid face is tracked in this face source
                                if (bodies[i].IsTracked)
                                {
                                    

                                    IReadOnlyDictionary<JointType, Joint> joints = bodies[i].Joints;

                                    // convert the joint points to depth (display) space
                                    Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                                    foreach (JointType jointType in joints.Keys)
                                    {
                                        // sometimes the depth(Z) of an inferred joint may show as negative
                                        // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                        CameraSpacePoint position = joints[jointType].Position;
                                        if (position.Z < 0)
                                        {
                                            position.Z = 0.1f;
                                        }

                                        ColorSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);
                                        jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                    }

                                    this.DrawBody(joints, jointPoints, dc, drawPen);


                                    // check if we have valid face frame results
                                    //if (faceFrameResults[i] != null)
                                    {
                                        int PolyOrder = 1;
                                        CameraSpacePoint head = bodies[i].Joints[JointType.Head].Position;
                                        // draw face frame results
                                        //PointF nosePoint = faceFrameResults[i].FacePointsInColorSpace[FacePointType.Nose];
                                        PointF headlocation = new PointF()
                                        {
                                            X = (float)jointPoints[JointType.Head].X,
                                            Y = (float)jointPoints[JointType.Head].Y
                                        };
                                        faceLocationBuffer[i].AddFirst(headlocation);
                                        if (faceLocationBuffer[i].Count > 4)
                                            faceLocationBuffer[i].RemoveLast();
                                        if (faceLocationBuffer[i].Count > PolyOrder+1)
                                        {
                                            drawCurve(faceLocationBuffer[i], dc, PolyOrder);
                                        }
                                        //this.DrawFaceFrameResults(i, this.faceFrameResults[i], dc);
                                        if (!drawFaceResult)
                                        {
                                            drawFaceResult = true;
                                        }
                                    }
                                }
                                else
                                {
                                    // check if the corresponding body is tracked 
                                    if (this.bodies[i].IsTracked)
                                    {
                                        // update the face frame source to track this body
                                        this.faceFrameSources[i].TrackingId = this.bodies[i].TrackingId;
                                    }
                                }
                            }

                            if (!drawFaceResult)
                            {
                                // if no faces were drawn then this indicates one of the following:
                                // a body was not tracked 
                                // a body was tracked but the corresponding face was not tracked
                                // a body and the corresponding face was tracked though the face box or the face points were not valid
                                dc.DrawText(this.textFaceNotTracked, this.textLayoutFaceNotTracked);
                            }
                        }
                        if (tracking == TrackingType.Circle || tracking == TrackingType.All)
                        {
                            //get Rectangle of interest
                            int ROIWidth = maxRadius*2;
                            int ROIHeight = maxRadius*2;
                            int ROICenterX;
                            int ROICenterY;
                            if (opencvCirclesHolder.Length > 0 || circleLocationBuffer.Count > 0 && circleLocationBuffer[0].Count > 0)
                            {
                                ROICenterX = (int)circleLocationBuffer[0].First.Value.X;
                                ROICenterY = (int)circleLocationBuffer[0].First.Value.Y;
                            }
                            else
                            {
                                ROICenterX = displayWidth / 2;
                                ROICenterY = displayHeight / 2;
                            }
                            int ROIXOffset = ROICenterX - ROIWidth / 2;
                            int ROIYOffset = ROICenterY - ROIHeight / 2;
                            //make sure that the square is in the frame lower bound
                            ROIXOffset = ROIXOffset < 0 ? 0 : ROIXOffset;
                            ROIYOffset = ROIYOffset < 0 ? 0 : ROIYOffset;
                            //make sure that the square is in the frame upper bound
                            ROIXOffset = ROIXOffset > displayWidth - ROIWidth - 1 ? displayWidth - ROIWidth - 1 : ROIXOffset;
                            ROIYOffset = ROIYOffset > displayHeight - ROIHeight - 1 ? displayHeight - ROIHeight - 1 : ROIYOffset;
                            //set ROI
                            OpenCvSharp.Rect ROI = new OpenCvSharp.Rect(ROIXOffset, ROIYOffset, ROIWidth, ROIHeight);
                            opencv8grey = new Mat(new Mat(new int[] { this.bitmap.PixelHeight, this.bitmap.PixelWidth * 4 }, MatType.CV_8UC1, this.bitmap.BackBuffer)
                            .Resize(new OpenCvSharp.Size(this.bitmap.PixelWidth, this.bitmap.PixelHeight), 0.25, 0), ROI);
                            
                            dc.DrawRectangle(null, new Pen(Brushes.Gray, 10), new System.Windows.Rect(ROIXOffset, ROIYOffset, ROI.Width, ROI.Height));
                            //skip a number of frames
                            if (frameCounter % 1 == 0)
                            {
                                // Add input data
                                //ModelInput input = new ModelInput();
                                //input.ImageSource = opencv8grey;
                                // Load model and predict output of sample data
                                //ModelOutput result = ConsumeModel.Predict(input);

                                opencvCirclesHolder = opencv8grey.HoughCircles(HoughMethods.GradientAlt, dp_resolution, minDistanceFromOtherCenter, canny_upper_threshold, confidence, minRadius, maxRadius);
                            }

                            //list of locations that have a circle drawn
                            List<System.Windows.Point> drawncircles = new List<System.Windows.Point>();
                            //if we detected more circles than the buffer is tracking
                            if(opencvCirclesHolder.Length > circleLocationBuffer.Count)
                            {
                                // add the new circles to the buffer list
                                for(int i = 0; i < opencvCirclesHolder.Length - circleLocationBuffer.Count; i++)
                                {
                                    circleLocationBuffer.Add(new LinkedList<PointF>());
                                }
                            }
                            //loop through the circles that we found
                            for (int i = 0; i < opencvCirclesHolder.Length; i++)
                            {
                                // current circle
                                CircleSegment circle = opencvCirclesHolder[i];
                                //check if we've drawn this circle before
                                if (!drawncircles.Contains(new System.Windows.Point(circle.Center.X, circle.Center.Y)))
                                {
                                    PointF circleCenter = new PointF();
                                    circleCenter.X = circle.Center.X + ROIXOffset;
                                    circleCenter.Y = circle.Center.Y + ROIYOffset;
                                    //add new circle location to the buffer
                                    circleLocationBuffer[i].AddFirst(circleCenter);
                                    //sets that max buffer size
                                    if (circleLocationBuffer[i].Count > 10)
                                        circleLocationBuffer[i].RemoveLast();
                                    int polyOrder = 2;
                                    //to make sure that the curve fitting algorithm has enough data points
                                    if (circleLocationBuffer[i].Count > polyOrder)
                                    {
                                        drawCurve(circleLocationBuffer[i], dc, polyOrder);
                                    }

                                    //dc.DrawRectangle(Brushes.BlueViolet, null, new System.Windows.Rect(new System.Windows.Point(circle.Center.X - 5, circle.Center.Y - 5), new System.Windows.Point(circle.Center.X + 5, circle.Center.Y + 5)));
                                    //draw the circle's confidence order index
                                    dc.DrawText(
                                        new FormattedText(i.ToString(), CultureInfo.GetCultureInfo("en-us"), FlowDirection.LeftToRight, new Typeface("Georgia"), DrawTextFontSize, Brushes.BlueViolet),
                                        new System.Windows.Point(circleCenter.X - DrawTextFontSize / 2, circleCenter.Y - DrawTextFontSize / 2)
                                    );
                                    //draw circle around circle
                                    dc.DrawEllipse(null, new Pen(Brushes.Cyan, 5), new System.Windows.Point(circleCenter.X, circleCenter.Y), circle.Radius, circle.Radius);
                                    //add circle to the list of drawn circles
                                    drawncircles.Add(new System.Windows.Point(circle.Center.X, circle.Center.Y));
                                }
                            }
                        }
                        //draw helper data in corner of screen
                        string data = $"FramesPerSecond: {FramesPerSecond}\ndp_resolution: {dp_resolution}\nminDistanceFromOtherCenter: {minDistanceFromOtherCenter}\ncanny_upper_threshold: {canny_upper_threshold}\nconfidence: {confidence*100}%\nminRadius: {minRadius}\nmaxRadius: {maxRadius}";
                        dc.DrawText(
                            new FormattedText(data, CultureInfo.GetCultureInfo("en-us"), FlowDirection.RightToLeft, new Typeface("Georgia"), DrawTextFontSize, Brushes.Blue),
                            new System.Windows.Point(bitmap.PixelWidth - 10, 10)
                        );
                    }
                }

            }
            finally
            {
                //update the screen
                this.drawingGroup.ClipGeometry = new RectangleGeometry(this.displayRect);
                this.bitmap.AddDirtyRect(new Int32Rect(0, 0, this.bitmap.PixelWidth, this.bitmap.PixelHeight));
                if (isBitmapLocked)
                {
                    bitmap.Unlock();
                }

                if (colorFrame != null)
                {
                    colorFrame.Dispose();
                }

                if (bodyFrame != null)
                {
                    bodyFrame.Dispose();
                }
                frameCounter += 1;
            }
        }
        /// <summary>
        /// converts the given bitmap to a bitmap source
        /// </summary>
        /// <param name="bitmap"></param>
        /// <returns>the given bitmap as a bitmap source</returns>
        private static BitmapSource Convert(System.Drawing.Bitmap bitmap)
        {
            var bitmapData = bitmap.LockBits(
                new System.Drawing.Rectangle(0, 0, bitmap.Width, bitmap.Height),
                System.Drawing.Imaging.ImageLockMode.ReadOnly, bitmap.PixelFormat);

            var bitmapSource = BitmapSource.Create(
                bitmapData.Width, bitmapData.Height,
                bitmap.HorizontalResolution, bitmap.VerticalResolution,
                PixelFormats.Gray8, null,
                bitmapData.Scan0, bitmapData.Stride * bitmapData.Height, bitmapData.Stride);

            bitmap.UnlockBits(bitmapData);
            return bitmapSource;
        }
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            using (var bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    // update body data
                    bodyFrame.GetAndRefreshBodyData(this.bodies);

                    using (DrawingContext dc = this.drawingGroup.Open())
                    {
                        // draw the dark background
                        dc.DrawRectangle(Brushes.Black, null, this.displayRect);
                        bool drawFaceResult = false;

                        // iterate through each face source
                        for (int i = 0; i < this.bodyCount; i++)
                        {
                            // check if a valid face is tracked in this face source
                            if (this.faceFrameSources[i].IsTrackingIdValid)
                            {
                                // check if we have valid face frame results
                                if (this.faceFrameResults[i] != null)
                                {
                                    // draw face frame results
                                    this.DrawFaceFrameResults(i, this.faceFrameResults[i], dc);

                                    if (!drawFaceResult)
                                    {
                                        drawFaceResult = true;
                                    }
                                }
                            }
                            else
                            {
                                // check if the corresponding body is tracked 
                                if (this.bodies[i].IsTracked)
                                {
                                    // update the face frame source to track this body
                                    this.faceFrameSources[i].TrackingId = this.bodies[i].TrackingId;
                                }
                            }
                        }

                        if (!drawFaceResult)
                        {
                            // if no faces were drawn then this indicates one of the following:
                            // a body was not tracked 
                            // a body was tracked but the corresponding face was not tracked
                            // a body and the corresponding face was tracked though the face box or the face points were not valid
                            dc.DrawText(this.textFaceNotTracked, this.textLayoutFaceNotTracked);
                        }

                        this.drawingGroup.ClipGeometry = new RectangleGeometry(this.displayRect);
                    }
                }
            }
        }
        /// <summary>
        /// draws a cross on the location of the circles saves an image of the matrix to C:\Users\Tevatron\Desktop\MatSC
        /// </summary>
        /// <param name="opencvCircles"></param>
        /// <param name="opencv8grey"></param>
        private void saveMat(CircleSegment[] opencvCircles, Mat opencv8grey)
        {

            foreach (CircleSegment circle in opencvCircles)
                opencv8grey.DrawMarker(new OpenCvSharp.Point(circle.Center.X, circle.Center.Y), new Scalar(128), MarkerTypes.Cross, (int)circle.Radius * 2, 10);
            int numpics = Directory.GetFiles(@"C:\Users\Tevatron\Desktop\MatSC", "*.png").Length;
            opencv8grey.ImWrite($@"C:\Users\Tevatron\Desktop\MatSC\image{numpics + 1}.png");
        }
        /// <summary>
        /// Draws face frame results
        /// </summary>
        /// <param name="faceIndex">the index of the face frame corresponding to a specific body in the FOV</param>
        /// <param name="faceResult">container of all face frame results</param>
        /// <param name="drawingContext">drawing context to render to</param>
        private void DrawFaceFrameResults(int faceIndex, FaceFrameResult faceResult, DrawingContext drawingContext)
        {
            // choose the brush based on the face index
            Brush drawingBrush = this.faceBrush[0];
            if (faceIndex < this.bodyCount)
            {
                drawingBrush = this.faceBrush[faceIndex];
            }

            Pen drawingPen = new Pen(drawingBrush, DrawFaceShapeThickness);

            // draw the face bounding box
            var faceBoxSource = faceResult.FaceBoundingBoxInColorSpace;
            System.Windows.Rect faceBox = new System.Windows.Rect(faceBoxSource.Left, faceBoxSource.Top, faceBoxSource.Right - faceBoxSource.Left, faceBoxSource.Bottom - faceBoxSource.Top);
            drawingContext.DrawRectangle(null, drawingPen, faceBox);

            if (faceResult.FacePointsInColorSpace != null)
            {
                // draw each face point
                foreach (PointF pointF in faceResult.FacePointsInColorSpace.Values)
                {
                    drawingContext.DrawEllipse(null, drawingPen, new System.Windows.Point(pointF.X, pointF.Y), FacePointRadius, FacePointRadius);
                }
            }

            string faceText = string.Empty;

            // extract each face property information and store it in faceText
            if (faceResult.FaceProperties != null)
            {
                foreach (var item in faceResult.FaceProperties)
                {
                    faceText += item.Key.ToString() + " : ";

                    // consider a "maybe" as a "no" to restrict 
                    // the detection result refresh rate
                    if (item.Value == DetectionResult.Maybe)
                    {
                        faceText += DetectionResult.No + "\n";
                    }
                    else
                    {
                        faceText += item.Value.ToString() + "\n";
                    }
                }
            }

            // extract face rotation in degrees as Euler angles
            if (faceResult.FaceRotationQuaternion != null)
            {
                int pitch, yaw, roll;
                ExtractFaceRotationInDegrees(faceResult.FaceRotationQuaternion, out pitch, out yaw, out roll);
                faceText += "FaceYaw : " + yaw + "\n" +
                            "FacePitch : " + pitch + "\n" +
                            "FacenRoll : " + roll + "\n";
            }

            // render the face property and face rotation information
            System.Windows.Point faceTextLayout;
            if (this.GetFaceTextPositionInColorSpace(faceIndex, out faceTextLayout))
            {
                //drawingContext.DrawText(new FormattedText(faceText,CultureInfo.GetCultureInfo("en-us"),FlowDirection.LeftToRight,new Typeface("Georgia"),DrawTextFontSize,drawingBrush),faceTextLayout);
            }
        }
        /// <summary>
        /// Computes the face result text position by adding an offset to the corresponding 
        /// body's head joint in camera space and then by projecting it to screen space
        /// </summary>
        /// <param name="faceIndex">the index of the face frame corresponding to a specific body in the FOV</param>
        /// <param name="faceTextLayout">the text layout position in screen space</param>
        /// <returns>success or failure</returns>
        private bool GetFaceTextPositionInColorSpace(int faceIndex, out System.Windows.Point faceTextLayout)
        {
            faceTextLayout = new System.Windows.Point();
            bool isLayoutValid = false;

            Body body = this.bodies[faceIndex];
            if (body.IsTracked)
            {
                var headJoint = body.Joints[JointType.Head].Position;

                CameraSpacePoint textPoint = new CameraSpacePoint()
                {
                    X = headJoint.X + TextLayoutOffsetX,
                    Y = headJoint.Y + TextLayoutOffsetY,
                    Z = headJoint.Z
                };

                ColorSpacePoint textPointInColor = this.coordinateMapper.MapCameraPointToColorSpace(textPoint);

                faceTextLayout.X = textPointInColor.X;
                faceTextLayout.Y = textPointInColor.Y;
                isLayoutValid = true;
            }

            return isLayoutValid;
        }
        /// <summary>
        /// Validates face bounding box and face points to be within screen space
        /// </summary>
        /// <param name="faceResult">the face frame result containing face box and points</param>
        /// <returns>success or failure</returns>
        private bool ValidateFaceBoxAndPoints(FaceFrameResult faceResult)
        {
            bool isFaceValid = faceResult != null;

            if (isFaceValid)
            {
                var faceBox = faceResult.FaceBoundingBoxInColorSpace;
                if (faceBox != null)
                {
                    // check if we have a valid rectangle within the bounds of the screen space
                    isFaceValid = (faceBox.Right - faceBox.Left) > 0 &&
                                  (faceBox.Bottom - faceBox.Top) > 0 &&
                                  faceBox.Right <= this.displayWidth &&
                                  faceBox.Bottom <= this.displayHeight;

                    if (isFaceValid)
                    {
                        var facePoints = faceResult.FacePointsInColorSpace;
                        if (facePoints != null)
                        {
                            foreach (PointF pointF in facePoints.Values)
                            {
                                // check if we have a valid face point within the bounds of the screen space
                                bool isFacePointValid = pointF.X > 0.0f &&
                                                        pointF.Y > 0.0f &&
                                                        pointF.X < this.displayWidth &&
                                                        pointF.Y < this.displayHeight;

                                if (!isFacePointValid)
                                {
                                    isFaceValid = false;
                                    break;
                                }
                            }
                        }
                    }
                }
            }

            return isFaceValid;
        }
        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            if (this.kinectSensor != null)
            {
                // on failure, set the status text
                this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                                : Properties.Resources.SensorNotAvailableStatusText;
            }
        }
        public bool dragging = false;
        /// <summary>
        /// sets the hough circle detection parameters
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Tracking_values_changed(object sender, double e)
        {
            switch ((sender as Slider).Name)
            {
                case "accumRes":
                    this.dp_resolution = (int)e;
                    break;
                case "circleDist":
                    this.minDistanceFromOtherCenter = (int)e;
                    using (DrawingContext dc = this.drawingGroup.Append())
                    {
                        dc.DrawEllipse(null, new Pen(Brushes.Cyan, 5), new Point((this.bitmap.PixelWidth / 2) - minDistanceFromOtherCenter/2, this.bitmap.PixelHeight / 2), minRadius, minRadius);
                        dc.DrawEllipse(null, new Pen(Brushes.Cyan, 5), new Point((this.bitmap.PixelWidth / 2) + minDistanceFromOtherCenter/2, this.bitmap.PixelHeight / 2), minRadius, minRadius);
                    }
                    break;
                case "cannyThreshold":
                    this.canny_upper_threshold = (int)e;
                    break;
                case "confidence":
                    this.confidence = (double)((int)e / 100.0);
                    break;
                case "minCircle":
                    this.minRadius = (int)e;
                    using (DrawingContext dc = this.drawingGroup.Append())
                    {
                        dc.DrawEllipse(null, new Pen(Brushes.Cyan, 5), new System.Windows.Point(this.bitmap.PixelWidth / 2, this.bitmap.PixelHeight / 2), e, e);
                    }
                    break;
                case "maxCircle":
                    this.maxRadius = (int)e;
                    using (DrawingContext dc = this.drawingGroup.Append())
                    {
                        dc.DrawEllipse(null, new Pen(Brushes.Cyan, 5), new System.Windows.Point(this.bitmap.PixelWidth / 2, this.bitmap.PixelHeight / 2), e, e);
                    }
                    break;
            }
        }

        /// <summary>
        /// sets the object type to track
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void Tracking_Type_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            ComboBoxItem btn = (sender as ComboBox).SelectedItem as ComboBoxItem;
            if (btn != null)
            {
                if ((btn.Content as String).Equals("Face"))
                {
                    tracking = TrackingType.Face;
                }
                else if ((btn.Content as String).Equals("Circle"))
                {
                    tracking = TrackingType.Circle;
                }
                else if ((btn.Content as String).Equals("All"))
                {
                    tracking = TrackingType.All;
                }
                else if ((btn.Content as String).Equals("None"))
                {
                    tracking = TrackingType.None;
                }
            }
        }
        /// <summary>
        /// opens the canny detection and hough circle detection adjuster window
        /// </summary>
        /// <param name="sender">the WPF object that the data 'e' is coming from</param>
        /// <param name="e">the data</param>
        private void Button_Click(object sender, RoutedEventArgs e)
        {
            HoughCircleAdjustWindow AdjustmentWindow = new HoughCircleAdjustWindow();
            AdjustmentWindow.CircleDetectionAdjusted += Tracking_values_changed;
            AdjustmentWindow.Owner = this;
            AdjustmentWindow.Show();
        }
        /// <summary>
        /// saves the canny view screen shot
        /// </summary>
        /// <param name="sender">not used</param>
        /// <param name="e">not used</param>
        private void Button_Click_1(object sender, RoutedEventArgs e)
        {
            Mat mat = new Mat(new int[] { this.bitmap.PixelHeight, this.bitmap.PixelWidth * 4 }, MatType.CV_8UC1, this.bitmap.BackBuffer)
                            .Resize(new OpenCvSharp.Size(this.bitmap.PixelWidth, this.bitmap.PixelHeight), 0.25, 0);
            saveMat(opencvCirclesHolder, mat);
        }

        private void dispatcherTimer_Tick(object sender, EventArgs e)
        {
            FramesPerSecond = (int)((frameCounter - prevFrameCounter) / stopwatch.Elapsed.TotalSeconds);
            prevFrameCounter = frameCounter;
            stopwatch.Restart();
        }
    }
    public static class LinkedListExtensions
    {
        public static T Get<T>(this LinkedList<T> list, int idx)
        {
            T data = default;
            int i = 0;
            foreach(T thing in list)
            {
                if (i == idx)
                {
                    data = thing;
                }
            }
            return data;
        }
    }
}

