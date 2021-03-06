﻿using T = Jp.Digitalmuseum.Kinect;
using Microsoft.Kinect;
using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Net;
using System.Text;
using System.Threading;
using Thrift.Collections;

namespace ConsoleKinectServer
{
    class KinectServiceHandler : T.KinectService.Iface
    {
        #region Private State
        private SpeechRecognitionEngine sre;
        private KinectAudioSource kinectAudioSource;
        private HashSet<string> words;
        private HashSet<string> keywords;
        private byte[] imageData;
        private short[] depthImageData;
        private byte[] depthImageRawData;
        private Skeleton[] skeletonData;
        private T.Frame frame;
        private bool voiceEnabled;
        private bool colorEnabled;
        private bool depthEnabled;
        #endregion Private State

        #region Thrift server implementation
        public KinectServiceHandler()
        {
            voiceEnabled = false;
            colorEnabled = true;
            depthEnabled = false;
            InitializeSpeechRecognition();
            KinectStart();
        }

        public bool isDeviceConnected()
        {
            return Kinect != null && Kinect.Status == KinectStatus.Connected;
        }

        public void setVoiceEnabled(bool isEnabled)
        {
            voiceEnabled = isEnabled;
            UpdateSpeechRecognition();
        }

        public bool isVoiceEnabled()
        {
            return true;
        }

        public void addKeyword(string text)
        {
            keywords.Add(text);
            UpdateSpeechRecognition();
        }

        public void removeKeyword(string text)
        {
            keywords.Remove(text);
            UpdateSpeechRecognition();
        }

        private void UpdateSpeechRecognition()
        {
            if (!isDeviceConnected())
            {
                return;
            }
            if (keywords.Count <= 0 || !voiceEnabled)
            {
                StopSpeechRecognition();
                return;
            }
            GrammarBuilder gb = new GrammarBuilder();

            // This is needed to ensure that it will work on machines with any culture, not just en-us.
            gb.Culture = sre.RecognizerInfo.Culture;

            string[] keywordsArray = keywords.ToArray();
            gb.Append(new Choices(keywordsArray));

            sre.LoadGrammarAsync(new Grammar(gb));

            StartSpeechRecognition(Kinect.AudioSource);
        }

        public THashSet<string> getKeywords()
        {
            var keywords = new THashSet<string>();
            foreach (string keyword in this.keywords)
            {
                keywords.Add(keyword);
            }
            return keywords;
        }

        public void setColorEnabled(bool isEnabled)
        {
            colorEnabled = isEnabled;
        }

        public bool isColorEnabled()
        {
            return colorEnabled;
        }

        public void setDepthEnabled(bool isEnabled)
        {
            depthEnabled = isEnabled;
        }

        public bool isDepthEnabled()
        {
            return depthEnabled;
        }

        private int currentAngle = int.MaxValue;
        private int targetAngle = int.MaxValue;
        public void setAngle(int angle)
        {
            if (!isDeviceConnected())
            {
                return;
            }
            if (currentAngle != int.MaxValue)
            {
                // Request the change.
                targetAngle = angle;
            }
            else
            {
                int a = angle;
                new Thread(() =>
                {
                    lock (Kinect)
                    {
                        currentAngle = Kinect.ElevationAngle;
                        while (isDeviceConnected())
                        {
                            Kinect.ElevationAngle = a;

                            // See http://social.msdn.microsoft.com/Forums/en-US/kinectsdknuiapi/thread/7ddf2f6e-0f6f-4dbe-87a0-69de1844e5f3/
                            Thread.Sleep(1400);

                            if (targetAngle == int.MaxValue)
                            {
                                // If there's no more pending request, get out of the loop.
                                break;
                            }
                            a = targetAngle;
                            targetAngle = int.MaxValue;
                        }
                        currentAngle = int.MaxValue;
                    }
                }).Start();
            }
        }

        public int getAngle()
        {
            if (!isDeviceConnected())
            {
                return int.MaxValue;
            }
            if (currentAngle != int.MaxValue)
            {
                return currentAngle;
            }
            return Kinect.ElevationAngle;
        }

        public T.Frame getFrame()
        {
            words.Clear();
            return frame;
        }

        public void shutdown()
        {
            if (Shutdown != null)
            {
                Shutdown();
            }
        }

        public delegate void ShutdownAction();
        public ShutdownAction Shutdown { internal set; get; }
        #endregion Thrift server implementation

        #region Kinect discovery + setup

        // Kinect enabled apps should customize each message and replace the technique of displaying the message.
        private void ShowStatus(ErrorCondition errorCondition)
        {
            string statusMessage;
            switch (errorCondition)
            {
                case ErrorCondition.None:
                    statusMessage = null;
                    break;
                case ErrorCondition.NoKinect:
                    statusMessage = Properties.Resources.NoKinectError;
                    break;
                case ErrorCondition.NoPower:
                    statusMessage = Properties.Resources.NoPowerError;
                    break;
                case ErrorCondition.NoSpeech:
                    statusMessage = Properties.Resources.NoSpeechError;
                    break;
                case ErrorCondition.NotReady:
                    statusMessage = Properties.Resources.NotReady;
                    break;
                case ErrorCondition.Initializing:
                    statusMessage = Properties.Resources.Initializing;
                    break;
                case ErrorCondition.KinectAppConflict:
                    statusMessage = Properties.Resources.KinectAppConflict;
                    break;
                case ErrorCondition.InsufficientBandwidth:
                    statusMessage = Properties.Resources.InsufficientBandwidth;
                    break;
                default:
                    statusMessage = errorCondition.ToString();
                    break;
            }
            Console.WriteLine(statusMessage);
            currentErrorCondition = errorCondition;
        }

        // Kinect enabled apps should customize which Kinect services it initializes here.
        private KinectSensor InitializeKinectServices(KinectSensor sensor)
        {
            // Application should enable all streams first.
            sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            sensor.DepthStream.Enable(DepthImageFormat.Resolution320x240Fps30);

            // Smoothed with some latency.
            // Filters out medium jitters.
            // Good for a menu system that needs to be smooth but
            // doesn't need the reduced latency as much as gesture recognition does.
            sensor.SkeletonStream.Enable(new TransformSmoothParameters()
            {
                // See http://msdn.microsoft.com/en-us/library/jj131024.aspx for details.
                Smoothing = 0.5f,
                Correction = 0.1f,
                Prediction = 0.5f,
                JitterRadius = 0.1f,
                MaxDeviationRadius = 0.1f
            });

            imageData = new byte[640 * 480 * 4];
            depthImageData = new short[320 * 240];
            depthImageRawData = new byte[320 * 240 * 2];

            sensor.AllFramesReady += new EventHandler<AllFramesReadyEventArgs>(AllFramesReady);

            try
            {
                sensor.Start();
            }
            catch (IOException)
            {
                ShowStatus(ErrorCondition.KinectAppConflict);
                return null;
            }

            UpdateSpeechRecognition();
            return sensor;
        }

        //Kinect enabled apps should uninitialize all Kinect services that were initialized in InitializeKinectServices() here.
        private void UninitializeKinectServices(KinectSensor sensor)
        {
            sensor.Stop();

            sensor.AllFramesReady -= new EventHandler<AllFramesReadyEventArgs>(AllFramesReady);

            StopSpeechRecognition();
        }

        #region Most apps won't modify this code
        private void KinectStart()
        {
            KinectDiscovery();
            if (Kinect == null)
            {
                if (KinectSensor.KinectSensors.Count == 0)
                {
                    ShowStatus(ErrorCondition.NoKinect);
                }
                else
                {
                    KinectStatus sensorStatus = KinectSensor.KinectSensors[0].Status;
                    switch (sensorStatus)
                    {
                        case KinectStatus.InsufficientBandwidth:
                            ShowStatus(ErrorCondition.InsufficientBandwidth);
                            break;
                        case KinectStatus.NotPowered:
                            ShowStatus(ErrorCondition.NoPower);
                            break;
                        default:
                            ShowStatus(ErrorCondition.NotReady);
                            break;
                    }
                }
            }
        }

        private void KinectStop()
        {
            if (Kinect != null)
            {
                Kinect = null;
            }
        }

        private bool IsKinectStarted
        {
            get { return Kinect != null; }
        }

        private void KinectDiscovery()
        {
            //listen to any status change for Kinects
            KinectSensor.KinectSensors.StatusChanged += new EventHandler<StatusChangedEventArgs>(Kinects_StatusChanged);

            //loop through all the Kinects attached to this PC, and start the first that is connected without an error.
            foreach (KinectSensor kinectSensor in KinectSensor.KinectSensors)
            {
                if (kinectSensor.Status == KinectStatus.Connected)
                {
                    if (Kinect == null)
                    {
                        Kinect = kinectSensor;
                        return;
                    }
                }
            }
        }

        private void Kinects_StatusChanged(object sender, StatusChangedEventArgs e)
        {
            switch (e.Status)
            {
                case KinectStatus.Initializing:
                    ShowStatus(ErrorCondition.Initializing);
                    break;
                case KinectStatus.Connected:
                    if (Kinect == null)
                    {
                        Kinect = e.Sensor; //if KinectSensor.Init() fails due to an AppDeviceConflict, this property will be null after return.
                        ShowStatus(ErrorCondition.None);
                    }
                    break;
                case KinectStatus.Disconnected:
                case KinectStatus.DeviceNotGenuine:
                    if (Kinect == e.Sensor)
                    {
                        Kinect = null;
                    }
                    break;
                case KinectStatus.NotReady:
                    if (Kinect == null)
                    {
                        ShowStatus(ErrorCondition.NotReady);
                    }
                    break;
                case KinectStatus.NotPowered:
                    if (Kinect == e.Sensor)
                    {
                        Kinect = null;
                        ShowStatus(ErrorCondition.NoPower);
                    }
                    break;
                case KinectStatus.InsufficientBandwidth:
                    if (Kinect == e.Sensor)
                    {
                        Kinect = null;
                        ShowStatus(ErrorCondition.InsufficientBandwidth);
                    }
                    break;
                default:
                    throw new Exception("Unhandled Status: " + e.Status);
            }
            if (Kinect == null)
            {
                ShowStatus(ErrorCondition.NoKinect);
            }
        }

        public KinectSensor Kinect
        {
            get
            {
                return _Kinect;
            }
            set
            {
                if (_Kinect != null)
                {
                    UninitializeKinectServices(_Kinect);
                }
                _Kinect = value;
                if (_Kinect != null)
                {
                    _Kinect = InitializeKinectServices(_Kinect);
                }
            }
        }

        private KinectSensor _Kinect;
        private ErrorCondition currentErrorCondition;

        internal enum ErrorCondition
        {
            None,
            Initializing,
            NoPower,
            NoKinect,
            NoSpeech,
            NotReady,
            KinectAppConflict,
            InsufficientBandwidth,
        }
        #endregion Most apps won't modify this code

        #endregion Kinect discovery + setup

        #region Speech recognition
        private void InitializeSpeechRecognition()
        {
            sre = new SpeechRecognitionEngine(GetKinectRecognizer());
            keywords = new HashSet<string>();
            words = new HashSet<string>();
        }

        private static RecognizerInfo GetKinectRecognizer()
        {
            Func<RecognizerInfo, bool> matchingFunc = r =>
            {
                string value;
                r.AdditionalInfo.TryGetValue("Kinect", out value);
                return "True".Equals(value, StringComparison.InvariantCultureIgnoreCase) && "en-US".Equals(r.Culture.Name, StringComparison.InvariantCultureIgnoreCase);
            };
            return SpeechRecognitionEngine.InstalledRecognizers().Where(matchingFunc).FirstOrDefault();
        }

        public void StartSpeechRecognition(KinectAudioSource kinectSource)
        {
            if (kinectAudioSource != null)
            {
                return;
            }

            kinectAudioSource = kinectSource;
            kinectAudioSource.AutomaticGainControlEnabled = false;
            kinectAudioSource.BeamAngleMode = BeamAngleMode.Adaptive;

            Stream kinectStream = kinectAudioSource.Start();
            sre.SetInputToAudioStream(kinectStream,
                new SpeechAudioFormatInfo(
                    EncodingFormat.Pcm, 16000, 16, 1, 32000, 2, null));
            sre.RecognizeAsync(RecognizeMode.Multiple);
            sre.SpeechRecognized += new EventHandler<SpeechRecognizedEventArgs>(sre_SpeechRecognized);
        }

        void sre_SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            words.Add(e.Result.Text);
        }

        public void StopSpeechRecognition()
        {
            if (kinectAudioSource != null)
            {
                kinectAudioSource.Stop();
                sre.RecognizeAsyncCancel();
                sre.RecognizeAsyncStop();
                kinectAudioSource = null;
            }
        }
        #endregion Speech recognition

        #region Kinect Skeleton processing
        void AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            // Construct data.
            T.Frame frame = new T.Frame();
            using (ColorImageFrame colorImageFrame = e.OpenColorImageFrame())
            using (DepthImageFrame depthImageFrame = e.OpenDepthImageFrame())
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (colorImageFrame == null)
                {
                    return;
                }
                frame.FrameId = colorImageFrame.FrameNumber;

                // Color image processing.
                if (colorEnabled)
                {
                    colorImageFrame.CopyPixelDataTo(imageData);
                    frame.Image = imageData;
                }

                // Depth image processing.
                if (depthImageFrame != null && depthEnabled)
                {
                    depthImageFrame.CopyPixelDataTo(depthImageData);
                    Buffer.BlockCopy(depthImageData, 0, depthImageRawData, 0, depthImageRawData.Length);
                    frame.DepthImage = depthImageRawData;
                }

                // Skeleton proccessing.
                frame.Joints = new Dictionary<T.JointType, T.Joint>();
                if (skeletonFrame != null)
                {
                    if ((skeletonData == null) || (skeletonData.Length != skeletonFrame.SkeletonArrayLength))
                    {
                        skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    }
                    skeletonFrame.CopySkeletonDataTo(skeletonData);
                    foreach (Skeleton skeleton in skeletonData)
                    {
                        if (SkeletonTrackingState.Tracked == skeleton.TrackingState)
                        {
                            // TODO Compare tracking id with skeleton.TrackingId
                            //Console.WriteLine("-------------");
                            //Console.WriteLine("tracked id: " + skeleton.TrackingId);
                            frame.Position = new T.Position3D();
                            frame.Position.X = skeleton.Position.X;
                            frame.Position.Y = skeleton.Position.Y;
                            frame.Position.Z = skeleton.Position.Z;
                            frame.Joints = new Dictionary<T.JointType, T.Joint>();
                            foreach (Joint joint in skeleton.Joints)
                            {
                                if (joint.TrackingState == JointTrackingState.Tracked)
                                {
                                    var j = new T.Joint();
                                    j.Position = new T.Position3D();
                                    j.Position.X = joint.Position.X;
                                    j.Position.Y = joint.Position.Y;
                                    j.Position.Z = joint.Position.Z;
                                    var sp = Kinect.CoordinateMapper.MapSkeletonPointToColorPoint(
                                        joint.Position, Kinect.ColorStream.Format);
                                    j.ScreenPosition = new T.Position2D();
                                    j.ScreenPosition.X = sp.X;
                                    j.ScreenPosition.Y = sp.Y;
                                    j.Type = ConvertJointType(joint.JointType);
                                    frame.Joints.Add(j.Type, j);
                                    //Console.WriteLine(j.Type + ": " + sp.X + ", " + sp.Y);
                                }
                            }
                            break;
                        }
                    }
                }
            }
            if (words.Count > 0)
            {
                frame.Words = new THashSet<string>();
                foreach (var word in words)
                {
                    frame.Words.Add(word);
                }
            }
            this.frame = frame;
        }

        private T.JointType ConvertJointType(JointType type)
        {
            switch (type)
            {
                case JointType.HipCenter:
                    return T.JointType.HIP_CENTER;
                case JointType.Spine:
                    return T.JointType.SPINE;
                case JointType.ShoulderCenter:
                    return T.JointType.SHOULDER_CENTER;
                case JointType.Head:
                    return T.JointType.HEAD;
                case JointType.ShoulderRight:
                    return T.JointType.SHOULDER_RIGHT;
                case JointType.ElbowRight:
                    return T.JointType.ELBOW_RIGHT;
                case JointType.WristRight:
                    return T.JointType.WRIST_RIGHT;
                case JointType.HandRight:
                    return T.JointType.HAND_RIGHT;
                case JointType.ShoulderLeft:
                    return T.JointType.SHOULDER_LEFT;
                case JointType.ElbowLeft:
                    return T.JointType.ELBOW_LEFT;
                case JointType.WristLeft:
                    return T.JointType.WRIST_LEFT;
                case JointType.HandLeft:
                    return T.JointType.HAND_LEFT;
                case JointType.HipRight:
                    return T.JointType.HIP_RIGHT;
                case JointType.KneeRight:
                    return T.JointType.KNEE_RIGHT;
                case JointType.AnkleRight:
                    return T.JointType.ANKLE_RIGHT;
                case JointType.FootRight:
                    return T.JointType.FOOT_RIGHT;
                case JointType.HipLeft:
                    return T.JointType.HIP_LEFT;
                case JointType.KneeLeft:
                    return T.JointType.KNEE_LEFT;
                case JointType.AnkleLeft:
                    return T.JointType.ANKLE_LEFT;
                case JointType.FootLeft:
                    return T.JointType.FOOT_LEFT;
                default:
                    return T.JointType.HEAD;
            }
        }
        #endregion Kinect Skeleton processing
    }
}
