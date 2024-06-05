//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Threading;
    using System.Windows.Controls;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using System.Windows.Media.Animation;
    using System.Linq;
    using Microsoft.Kinect;
    using System.Windows.Input;

    using System.Media;
    using System.Windows.Media;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        private Random random = new Random();
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        private const float boneOffset = -100;
        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(128, 0, 255, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Blue;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

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
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        /// 

        private System.Windows.Threading.DispatcherTimer gameTimer;
        private int gameDuration = 60; // 遊戲時間為60秒
        private int remainingTime;

        private MediaPlayer BGM;
        private MediaPlayer SFX1;
        private MediaPlayer SFX2;

        private ColorFrameReader colorFrameReader;
        private WriteableBitmap bitmap;

        public MainWindow()
        {
            //音樂和音效程式碼開始
            BGM = new MediaPlayer();
            SFX1 = new MediaPlayer();
            SFX2 = new MediaPlayer();
            string baseDirectory = AppDomain.CurrentDomain.BaseDirectory;
            string projectRootDirectory = Directory.GetParent(baseDirectory).Parent.Parent.Parent.FullName;
            string BGMPath = Path.Combine(projectRootDirectory, "Sounds", "BGM.wav");
            string SFX1Path = Path.Combine(projectRootDirectory, "Sounds", "SFX1.wav");
            string SFX2Path = Path.Combine(projectRootDirectory, "Sounds", "SFX2.wav");
            //Console.WriteLine(soundFilePath);
            BGM.Open(new Uri(BGMPath, UriKind.RelativeOrAbsolute));
            SFX1.Open(new Uri(SFX1Path, UriKind.RelativeOrAbsolute));
            SFX2.Open(new Uri(SFX2Path, UriKind.RelativeOrAbsolute));
            //音樂和音效程式碼結束

            this.WindowState = WindowState.Maximized;
            this.WindowStyle = WindowStyle.None;
            InitializeComponent();

            this.MainCanvas.MouseLeftButtonDown += MainCanvas_MouseLeftButtonDown;

            gameTimer = new System.Windows.Threading.DispatcherTimer();
            gameTimer.Tick += GameTimer_Tick;
            gameTimer.Interval = new TimeSpan(0, 0, 1);

            System.Windows.Threading.DispatcherTimer dispatcherTimer = new System.Windows.Threading.DispatcherTimer();
            dispatcherTimer.Tick += new EventHandler(show);
            dispatcherTimer.Interval = new TimeSpan(0, 0, 1);
            dispatcherTimer.Start();



            // one sensor is currently supported
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

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

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.Blue, 10));
            this.bodyColors.Add(new Pen(Brushes.Blue, 10));
            this.bodyColors.Add(new Pen(Brushes.Blue, 10));
            this.bodyColors.Add(new Pen(Brushes.Blue, 10));
            this.bodyColors.Add(new Pen(Brushes.Blue, 10));
            this.bodyColors.Add(new Pen(Brushes.Blue, 10));

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            this.colorFrameReader = kinectSensor.ColorFrameSource.OpenReader();
            this.colorFrameReader.FrameArrived += ColorFrameReader_FrameArrived;
            var frameDescription2 = kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);
            this.bitmap = new WriteableBitmap(frameDescription2.Width, frameDescription2.Height, 96.0, 96.0, System.Windows.Media.PixelFormats.Bgr32, null);
            kinectImage1.Source = bitmap;
            kinectImage1.Loaded += (s, e) =>
            {
                Canvas.SetLeft(kinectImage1, 0);
                Canvas.SetBottom(kinectImage1, 0);
            };
            kinectImage2.Source = bitmap;
            kinectImage2.Loaded += (s, e) =>
            {
                Canvas.SetLeft(kinectImage2, 0);
                Canvas.SetBottom(kinectImage2, 0);
            };
            kinectImage3.Source = bitmap;
            kinectImage3.Loaded += (s, e) =>
            {
                Canvas.SetLeft(kinectImage3, 0);
                Canvas.SetBottom(kinectImage3, 0);
            };

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        protected override void OnClosed(EventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }

            base.OnClosed(e);
        }

        private void ColorFrameReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            using (var frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    using (var buffer = frame.LockRawImageBuffer())
                    {
                        this.bitmap.Lock();
                        frame.CopyConvertedFrameDataToIntPtr(
                            this.bitmap.BackBuffer,
                            (uint)(this.bitmap.BackBufferStride * this.bitmap.PixelHeight),
                            ColorImageFormat.Bgra);

                        this.bitmap.AddDirtyRect(new Int32Rect(0, 0, this.bitmap.PixelWidth, this.bitmap.PixelHeight));
                        this.bitmap.Unlock();
                    }
                }
            }
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

        private void GameTimer_Tick(object sender, EventArgs e)
        {
            remainingTime--;
            TimeRemainingTextBlock.Text = $"{remainingTime}s";

            if (remainingTime <= 0)
            {
                EndGame();
            }
        }

        private void StartButton_Click(object sender, RoutedEventArgs e)
        {
            BGM.Position = TimeSpan.Zero;
            StartGame();
        }
        //private void MediaPlayer_MediaEnded(object sender, EventArgs e)
        //{
        //    BGM.Position = TimeSpan.Zero;
        //    BGM.Play();
        //}
        private void StartGame()
        {
            if (BGM.Position == TimeSpan.Zero)
            {
                BGM.Play();
            }
            
            //BGM.MediaEnded += MediaPlayer_MediaEnded;

            score = 0;
            ScoreTextBox.Text = "SCORE: 0";

            remainingTime = gameDuration;
            gameTimer.Start();

            StartButton.IsEnabled = false; // 禁用開始按鈕
        }


        private void EndGame()
        {
            gameTimer.Stop();

            while (imagesOnCanvas.Count > 0)
            {
                Image image = imagesOnCanvas[0];
                MainCanvas.Children.Remove(image);
                imagesOnCanvas.Remove(image);
                imagePositions.Remove(image);
            }
            
            MessageBoxResult result = MessageBox.Show($"遊戲結束！您的最終分數是 {score} 分。按下OK以再來一局。", "遊戲結束", MessageBoxButton.OK);

            if (result == MessageBoxResult.OK)
            {
                BGM.Position = TimeSpan.Zero;
                StartButton.IsEnabled = false;
                StartGame();
            }
            //else
            //{
            //    while (imagesOnCanvas.Count > 0)
            //    {
            //        Image image = imagesOnCanvas[0];
            //        MainCanvas.Children.Remove(image);
            //        imagesOnCanvas.Remove(image);
            //        imagePositions.Remove(image);
            //    }
            //}

            //StartButton.IsEnabled = true; // 啟用開始按鈕
            gmaestarted = false; // 重置右手狀態
        }



        private void MainCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            // 取得滑鼠點擊的位置
            Point clickPosition = e.GetPosition(this.MainCanvas);

            // 檢查是否有圖片被點擊
            CheckClickedImage(clickPosition);
        }
        private void CheckClickedImage(Point clickPosition)
        {
            foreach (Image image in imagesOnCanvas)
            {
                Rect imageRect = new Rect(imagePositions[image], new Size(image.Width, image.Height));
                if (imageRect.Contains(clickPosition))
                {
                    // 圖片被觸碰
                    IncreaseScore(10); // 增加 10 分
                    MainCanvas.Children.Remove(image);
                    imagesOnCanvas.Remove(image);
                    imagePositions.Remove(image);
                    break;
                }
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
        /// Execute start up tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
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

        private void CheckCollision(CameraSpacePoint handPosition, HandState handState, JointType handJointType)
        {
            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(handPosition);
            Point handPoint = new Point(depthSpacePoint.X, depthSpacePoint.Y);

            // 將手部位置進行相同的調整和放大
            double centerX = MainCanvas.ActualWidth / 2 + boneOffset;
            double centerY = MainCanvas.ActualHeight / 2;
            double scaledX = centerX + (handPoint.X - centerX) * scaleFactor;
            double scaledY = centerY + (handPoint.Y - centerY) * scaleFactor;
            Point scaledHandPoint = new Point(scaledX, scaledY);

            foreach (Image image in imagesOnCanvas)
            {
                Rect imageRect = new Rect(imagePositions[image], new Size(image.Width, image.Height));
                if (imageRect.Contains(scaledHandPoint))
                {
                    // 圖片被觸碰
                    string imageName = ((BitmapImage)image.Source).UriSource.Segments.Last();
                    if (handJointType == JointType.HandLeft && imageName == "franch.png")
                    {
                        // 左手觸碰到 french 類型圖片
                        IncreaseScore(10); // 增加 10 分
                        SFX1.Position = TimeSpan.Zero;
                        SFX1.Play();
                    }
                    else if (handJointType == JointType.HandRight && imageName == "ball.png")
                    {
                        // 右手觸碰到 ball 類型圖片
                        IncreaseScore(10); // 增加 10 分
                        SFX1.Position = TimeSpan.Zero;
                        SFX1.Play();
                    }
                    else
                    {
                        // 其他情況
                        IncreaseScore(-5); // 扣 5 分
                        SFX2.Position = TimeSpan.Zero;
                        SFX2.Play();
                    }

                    MainCanvas.Children.Remove(image);
                    imagesOnCanvas.Remove(image);
                    imagePositions.Remove(image);
                    break;
                }
            }
        }

        private bool gmaestarted = false;


        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;

            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }

            if (dataReceived)
            {
                this.SkeletonCanvas.Children.Clear(); // 清空之前的骨架
                foreach (Body body in this.bodies)
                {
                    if (body.IsTracked)
                    {
                        HandState leftHandState = body.HandLeftState;
                        if (leftHandState == HandState.Closed)
                        {
                            // 左手握拳
                            // 在左手位置繪製一個紅色圓圈
                            DrawHandPosition(body.Joints[JointType.HandLeft].Position, Brushes.Red);
                            // 檢查是否有圖片被觸碰
                            CheckCollision(body.Joints[JointType.HandLeft].Position, leftHandState, JointType.HandLeft);
                        }
                        else
                        {
                            // 左手不是握拳
                            // 在左手位置繪製一個綠色圓圈
                            DrawHandPosition(body.Joints[JointType.HandLeft].Position, Brushes.Green);
                        }

                        if (!gmaestarted && body.Joints[JointType.HandRight].Position.Y - body.Joints[JointType.Head].Position.Y > 0.2f)
                        {
                            gmaestarted = true;
                            StartGame(); // 開始遊戲
                        }

                        // 檢查右手狀態
                        HandState rightHandState = body.HandRightState;
                        if (rightHandState == HandState.Closed)
                        {
                            // 右手握拳
                            // 在右手位置繪製一個紅色圓圈
                            DrawHandPosition(body.Joints[JointType.HandRight].Position, Brushes.Red);
                            // 檢查是否有圖片被觸碰
                            CheckCollision(body.Joints[JointType.HandRight].Position, rightHandState, JointType.HandRight);
                        }
                        else
                        {
                            // 右手不是握拳
                            // 在右手位置繪製一個綠色圓圈
                            DrawHandPosition(body.Joints[JointType.HandRight].Position, Brushes.Green);
                        }

                        // 創建 jointPoints 字典
                        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                        foreach (JointType jointType in body.Joints.Keys)
                        {
                            CameraSpacePoint position = body.Joints[jointType].Position;
                            if (position.Z < 0)
                            {
                                position.Z = InferredZPositionClamp;
                            }
                            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                            jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                        }

                        this.DrawBody(body.Joints, jointPoints, this.SkeletonCanvas, this.bodyColors[0]);
                    }
                }
            }
        }

        private void DrawHandPosition(CameraSpacePoint handPosition, Brush handBrush)
        {
            DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(handPosition);
            Point handPoint = new Point(depthSpacePoint.X, depthSpacePoint.Y);

            double centerX = MainCanvas.ActualWidth / 2 + boneOffset;
            double centerY = MainCanvas.ActualHeight / 2;
            double scaledX = centerX + (handPoint.X - centerX) * scaleFactor;
            double scaledY = centerY + (handPoint.Y - centerY) * scaleFactor;

            System.Windows.Shapes.Ellipse handCircle = new System.Windows.Shapes.Ellipse
            {
                Width = 40,
                Height = 40,
                Fill = handBrush,
                Stroke = Brushes.Black,
                StrokeThickness = 2
            };
            Canvas.SetLeft(handCircle, scaledX - 10);
            Canvas.SetTop(handCircle, scaledY - 10);
            Canvas.SetZIndex(handCircle, int.MaxValue);
            this.SkeletonCanvas.Children.Add(handCircle);
        }


        private double scaleFactor = 1.5; // 調整此值以更改骨架大小


        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, Canvas canvas, Pen drawingPen)
        {
            double canvasWidth = canvas.ActualWidth;
            double canvasHeight = canvas.ActualHeight;
            double centerX = canvasWidth / 2 + boneOffset;
            double centerY = canvasHeight / 2;
            Dictionary<JointType, Point> jointPositions = new Dictionary<JointType, Point>();
            // 繪製骨架
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, canvas, drawingPen, centerX, centerY, this.scaleFactor);
            }

            // 繪製關節
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
                    System.Windows.Shapes.Ellipse joint = new System.Windows.Shapes.Ellipse
                    {
                        Width = JointThickness * this.scaleFactor,
                        Height = JointThickness * this.scaleFactor,
                        Fill = drawBrush
                    };
                    Canvas.SetLeft(joint, centerX + (jointPoints[jointType].X - centerX) * this.scaleFactor);
                    Canvas.SetTop(joint, centerY + (jointPoints[jointType].Y - centerY) * this.scaleFactor);
                    canvas.Children.Add(joint);
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
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, Canvas canvas, Pen drawingPen, double centerX, double centerY, double scaleFactor)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            if (joint0.TrackingState == TrackingState.NotTracked || joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = new Pen(drawingPen.Brush, drawingPen.Thickness * scaleFactor);
            }

            System.Windows.Shapes.Line bone = new System.Windows.Shapes.Line
            {
                X1 = centerX + (jointPoints[jointType0].X - centerX) * scaleFactor,
                Y1 = centerY + (jointPoints[jointType0].Y - centerY) * scaleFactor,
                X2 = centerX + (jointPoints[jointType1].X - centerX) * scaleFactor,
                Y2 = centerY + (jointPoints[jointType1].Y - centerY) * scaleFactor,
                Stroke = drawPen.Brush,
                StrokeThickness = drawPen.Thickness
            };
            canvas.Children.Add(bone);
        }



        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext)
        {
            switch (handState)
            {
                case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Open:
                    drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);
                    break;
            }
        }

        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
        //球出現
        public Image last = new Image();

        private List<Image> imagesOnCanvas = new List<Image>(); // 追踪畫面上的 Image 物件
        private Dictionary<Image, Point> imagePositions = new Dictionary<Image, Point>(); // 儲存 Image 位置

        private void show(object sender, EventArgs e)
        {
            if (remainingTime <= 0) return; // 如果遊戲已結束，則不顯示新圖片

            Image randomImage = new Image();
            string[] imageFiles = { "ball.png", "bomb.png", "franch.png" };
            int index;
            double randomX, randomY;

            // 確保新圖片的位置不會與現有圖片重疊
            bool isOverlapping;
            do
            {
                // 調整圖片生成的機率
                double rand = random.NextDouble();
                if (rand < 0.4)
                {
                    index = 0; // 生成 ball.png
                }
                else if (rand < 0.8)
                {
                    index = 2; // 生成 franch.png
                }
                else
                {
                    index = 1; // 生成 bomb.png
                }

                randomImage.Source = new BitmapImage(new Uri($"pack://application:,,,/Images/{imageFiles[index]}"));
                double canvasWidth = MainCanvas.ActualWidth;
                double canvasHeight = MainCanvas.ActualHeight;

                randomX = random.Next(-250, (int)canvasWidth - 650);
                randomY = random.Next(-200, (int)canvasHeight - 600);
                isOverlapping = false;
                foreach (var imagePos in imagePositions.Values)
                {
                    if (Math.Abs(Math.Abs(imagePos.X) - Math.Abs(randomX)) < 100 && Math.Abs(Math.Abs(imagePos.Y) - Math.Abs(randomY)) < 100)
                    {
                        isOverlapping = true;
                        break;
                    }
                }
            } while (isOverlapping);

            randomImage.Width = 100;
            randomImage.Height = 100;
            Canvas.SetLeft(randomImage, randomX);
            Canvas.SetTop(randomImage, randomY);

            MainCanvas.Children.Add(randomImage);
            imagesOnCanvas.Add(randomImage);
            imagePositions[randomImage] = new Point(randomX, randomY);

            DoubleAnimation fadeInAnimation = new DoubleAnimation
            {
                From = 0.0,
                To = 1.0,
                Duration = new Duration(TimeSpan.FromSeconds(0.3))
            };
            DoubleAnimation fadeOutAnimation = new DoubleAnimation
            {
                From = 1.0,
                To = 0.0,
                Duration = new Duration(TimeSpan.FromSeconds(0.3)),
                BeginTime = TimeSpan.FromSeconds(5)
            };

            Storyboard storyboard = new Storyboard();
            storyboard.Children.Add(fadeInAnimation);
            storyboard.Children.Add(fadeOutAnimation);

            Storyboard.SetTarget(fadeInAnimation, randomImage);
            Storyboard.SetTargetProperty(fadeInAnimation, new PropertyPath("Opacity"));
            Storyboard.SetTarget(fadeOutAnimation, randomImage);
            Storyboard.SetTargetProperty(fadeOutAnimation, new PropertyPath("Opacity"));

            fadeOutAnimation.Completed += (senderCompleted, eventArgs) =>
            {
                MainCanvas.Children.Remove(randomImage);
                imagesOnCanvas.Remove(randomImage);
                imagePositions.Remove(randomImage);
            };

            storyboard.Begin();


        }




        private int score = 0;

        private void IncreaseScore(int points)
        {
            score += points;
            ScoreTextBox.Text = "SCORE:" + score.ToString();
        }
        private void TextBox_TextChanged(object sender, TextChangedEventArgs e)
        {

        }

    }
}