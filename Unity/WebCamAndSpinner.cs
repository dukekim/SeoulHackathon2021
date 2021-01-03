#if !(PLATFORM_LUMIN && !UNITY_EDITOR)

using OpenCVForUnity.CoreModule;
using OpenCVForUnity.ImgprocModule;
using OpenCVForUnity.UnityUtils;
using System;
using System.Collections;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.EventSystems;
using UnityEngine.UI;


/// <summary>
/// WebCamTextureToMat Example
/// An example of converting a WebCamTexture image to OpenCV's Mat format.
/// </summary>
public class WebCamAndSpinner : MonoBehaviour
{
    /// <summary>
    /// Set the name of the device to use.
    /// </summary>
    [SerializeField, TooltipAttribute("Set the name of the device to use.")]
    public string requestedDeviceName = null;

    /// <summary>
    /// Set the width of WebCamTexture.
    /// </summary>
    [SerializeField, TooltipAttribute("Set the width of WebCamTexture.")]
    public int requestedWidth = 640;

    /// <summary>
    /// Set the height of WebCamTexture.
    /// </summary>
    [SerializeField, TooltipAttribute("Set the height of WebCamTexture.")]
    public int requestedHeight = 480;

    /// <summary>
    /// Set FPS of WebCamTexture.
    /// </summary>
    [SerializeField, TooltipAttribute("Set FPS of WebCamTexture.")]
    public int requestedFPS = 30;

    /// <summary>
    /// Set whether to use the front facing camera.
    /// </summary>
    [SerializeField, TooltipAttribute("Set whether to use the front facing camera.")]
    public bool requestedIsFrontFacing = false;

    /// <summary>
    /// The webcam texture.
    /// </summary>
    WebCamTexture webCamTexture;

    /// <summary>
    /// The webcam device.
    /// </summary>
    WebCamDevice webCamDevice;

    /// <summary>
    /// The rgba mat.
    /// </summary>
    Mat rgbaMat;
    Mat roiMat;
    Mat greyMat;
    Mat thresholdMat;
    Mat cannyMat;
    Mat colorEdges;
    Mat newThreshold;

    /// <summary>
    /// The colors.
    /// </summary>
    Color32[] colors;

    /// <summary>
    /// The texture.
    /// </summary>
    Texture2D texture;

    /// <summary>
    /// Indicates whether this instance is waiting for initialization to complete.
    /// </summary>
    bool isInitWaiting = false;

    /// <summary>
    /// Indicates whether this instance has been initialized.
    /// </summary>
    bool hasInitDone = false;

    /// <summary>
    /// The FPS monitor.
    /// </summary>
    //FpsMonitor fpsMonitor;

    /// <summary>
    /// Threshold Min & Max
    /// </summary>
    [Range(0, 255f)]
    public float thresholdMin = 175;
    [Range(0, 255f)]
    public float thresholdMax = 255;

    /// <summary>
    /// The objects.
    /// </summary>
    MatOfRect2d objects;

    /// <summary>
    /// The stored touch point.
    /// </summary>
    Point storedTouchPoint;

    /// <summary>
    /// Canny Min & Max
    /// </summary>
    [Range(0, 255f)]
    public float cannyMin = 255;
    public int ratio = 5;
    public int kernel_size = 3;

    /// <summary>
    /// ROI 설정영역
    /// </summary>
    Vector3 startScreenPos;
    Vector3 endScreenPos;
    private bool roiSelected = false;
    public int roiStartX = 72;
    public int roiStartY = 92;
    public int roiEndX = 543;
    public int roiEndY = 422;
    public int roiWidth = 466;
    public int roiHeight = 330;
    int roiCount255, roiCount0;

    /// <summary>
    /// 물체를 감지했을 경우 사용하는 변수들
    /// </summary>
    public bool isOpened = false;
    public GameObject luckyButtonUI;
    public GameObject clickIcon;
    private float currentTime;
    public float openTime = 3;
    public static WebCamAndSpinner instance;

    private void Awake()
    {
        if (instance == null)
            instance = this;
    }

    // Use this for initialization
    void Start()
    {
        //fpsMonitor = GetComponent<FpsMonitor>();

        luckyButtonUI.SetActive(false);
        clickIcon.SetActive(false);

        Initialize(); 
    }


    /// <summary>
    /// Initializes webcam texture.
    /// </summary>
    private void Initialize()
    {
        if (isInitWaiting)
            return;

#if UNITY_ANDROID && !UNITY_EDITOR
        // Set the requestedFPS parameter to avoid the problem of the WebCamTexture image becoming low light on some Android devices (e.g. Google Pixel, Pixel2).
        // https://forum.unity.com/threads/android-webcamtexture-in-low-light-only-some-models.520656/
        // https://forum.unity.com/threads/released-opencv-for-unity.277080/page-33#post-3445178
        if (requestedIsFrontFacing)
        {
            int rearCameraFPS = requestedFPS;
            requestedFPS = 15;
            StartCoroutine(_Initialize());
            requestedFPS = rearCameraFPS;
        }
        else
        {
            StartCoroutine(_Initialize());
        }
#else
        StartCoroutine(_Initialize());
#endif
    }

    /// <summary>
    /// Initializes webcam texture by coroutine.
    /// </summary>
    private IEnumerator _Initialize()
    {
        if (hasInitDone)
            Dispose();

        isInitWaiting = true;

        // Checks camera permission state.
#if UNITY_IOS && UNITY_2018_1_OR_NEWER
        UserAuthorization mode = UserAuthorization.WebCam;
        if (!Application.HasUserAuthorization(mode))
        {
            isUserRequestingPermission = true;
            yield return Application.RequestUserAuthorization(mode);

            float timeElapsed = 0;
            while (isUserRequestingPermission)
            {
                if (timeElapsed > 0.25f)
                {
                    isUserRequestingPermission = false;
                    break;
                }
                timeElapsed += Time.deltaTime;

                yield return null;
            }
        }

        if (!Application.HasUserAuthorization(mode))
        {
            if (fpsMonitor != null)
            {
                fpsMonitor.consoleText = "Camera permission is denied.";
            }
            isInitWaiting = false;
            yield break;
        }
#elif UNITY_ANDROID && UNITY_2018_3_OR_NEWER
        string permission = UnityEngine.Android.Permission.Camera;
        if (!UnityEngine.Android.Permission.HasUserAuthorizedPermission(permission))
        {
            isUserRequestingPermission = true;
            UnityEngine.Android.Permission.RequestUserPermission(permission);

            float timeElapsed = 0;
            while (isUserRequestingPermission)
            {
                if (timeElapsed > 0.25f)
                {
                    isUserRequestingPermission = false;
                    break;
                }
                timeElapsed += Time.deltaTime;

                yield return null;
            }
        }

        if (!UnityEngine.Android.Permission.HasUserAuthorizedPermission(permission))
        {
            //if (fpsMonitor != null)
            //{
            //    fpsMonitor.consoleText = "Camera permission is denied.";
            //}
            //isInitWaiting = false;
            yield break;
        }
#endif

        // Creates the camera
        var devices = WebCamTexture.devices;
        if (!String.IsNullOrEmpty(requestedDeviceName))
        {
            int requestedDeviceIndex = -1;
            if (Int32.TryParse(requestedDeviceName, out requestedDeviceIndex))
            {
                if (requestedDeviceIndex >= 0 && requestedDeviceIndex < devices.Length)
                {
                    webCamDevice = devices[requestedDeviceIndex];
                    webCamTexture = new WebCamTexture(webCamDevice.name, requestedWidth, requestedHeight, requestedFPS);
                }
            }
            else
            {
                for (int cameraIndex = 0; cameraIndex < devices.Length; cameraIndex++)
                {
                    if (devices[cameraIndex].name == requestedDeviceName)
                    {
                        webCamDevice = devices[cameraIndex];
                        webCamTexture = new WebCamTexture(webCamDevice.name, requestedWidth, requestedHeight, requestedFPS);
                        break;
                    }
                }
            }
            if (webCamTexture == null)
                Debug.Log("Cannot find camera device " + requestedDeviceName + ".");
        }

        if (webCamTexture == null)
        {
            // Checks how many and which cameras are available on the device
            for (int cameraIndex = 0; cameraIndex < devices.Length; cameraIndex++)
            {
#if UNITY_2018_3_OR_NEWER
                if (devices[cameraIndex].kind != WebCamKind.ColorAndDepth && devices[cameraIndex].isFrontFacing == requestedIsFrontFacing)
#else
                if (devices[cameraIndex].isFrontFacing == requestedIsFrontFacing)
#endif
                {
                    webCamDevice = devices[cameraIndex];
                    webCamTexture = new WebCamTexture(webCamDevice.name, requestedWidth, requestedHeight, requestedFPS);
                    break;
                }
            }
        }

        if (webCamTexture == null)
        {
            if (devices.Length > 0)
            {
                webCamDevice = devices[0];
                webCamTexture = new WebCamTexture(webCamDevice.name, requestedWidth, requestedHeight, requestedFPS);
            }
            else
            {
                Debug.LogError("Camera device does not exist.");
                isInitWaiting = false;
                yield break;
            }
        }

        // Starts the camera.
        webCamTexture.Play();

        while (true)
        {
            if (webCamTexture.didUpdateThisFrame)
            {
                Debug.Log("name:" + webCamTexture.deviceName + " width:" + webCamTexture.width + " height:" + webCamTexture.height + " fps:" + webCamTexture.requestedFPS);
                Debug.Log("videoRotationAngle:" + webCamTexture.videoRotationAngle + " videoVerticallyMirrored:" + webCamTexture.videoVerticallyMirrored + " isFrongFacing:" + webCamDevice.isFrontFacing);

                isInitWaiting = false;
                hasInitDone = true;

                OnInited();

                break;
            }
            else
            {
                yield return null;
            }
        }
    }

#if (UNITY_IOS && UNITY_2018_1_OR_NEWER) || (UNITY_ANDROID && UNITY_2018_3_OR_NEWER)
    bool isUserRequestingPermission;
        

    IEnumerator OnApplicationFocus(bool hasFocus)
    {
        yield return null;

        if (isUserRequestingPermission && hasFocus)
            isUserRequestingPermission = false;
    }
#endif

    /// <summary>
    /// Releases all resource.
    /// </summary>
    private void Dispose()
    {
        isInitWaiting = false;
        hasInitDone = false;

        if (webCamTexture != null)
        {
            webCamTexture.Stop();
            WebCamTexture.Destroy(webCamTexture);
            webCamTexture = null;
        }
        if (rgbaMat != null)
        {
            rgbaMat.Dispose();
            rgbaMat = null;
        }
        if (texture != null)
        {
            Texture2D.Destroy(texture);
            texture = null;
        }
    }

    /// <summary>
    /// Raises the webcam texture initialized event.
    /// </summary>
    private void OnInited()
    {
        if (colors == null || colors.Length != webCamTexture.width * webCamTexture.height)
            colors = new Color32[webCamTexture.width * webCamTexture.height];
        if (texture == null || texture.width != webCamTexture.width || texture.height != webCamTexture.height)
            texture = new Texture2D(webCamTexture.width, webCamTexture.height, TextureFormat.RGBA32, false);

        //rgbaMat = new Mat(webCamTexture.height, webCamTexture.width, CvType.CV_8UC4, new Scalar(0, 0, 0, 255));
        //Utils.matToTexture2D(rgbaMat, texture, colors);

        roiMat = new Mat(webCamTexture.height/2, webCamTexture.width/2, CvType.CV_8UC1);
        greyMat = new Mat(webCamTexture.height, webCamTexture.width, CvType.CV_8UC1);
        thresholdMat = new Mat(webCamTexture.height, webCamTexture.width, CvType.CV_8UC1);
        cannyMat = new Mat(webCamTexture.height, webCamTexture.width, CvType.CV_8UC1);
        colorEdges = new Mat(webCamTexture.height, webCamTexture.width, CvType.CV_8UC4, new Scalar(0, 0, 0, 255));
        Utils.matToTexture2D(greyMat, texture);



        gameObject.GetComponent<Renderer>().material.mainTexture = texture;

        gameObject.transform.localScale = new Vector3(webCamTexture.width, webCamTexture.height, 1);
        Debug.Log("Screen.width " + Screen.width + " Screen.height " + Screen.height + " Screen.orientation " + Screen.orientation);


        // FPS Monitor OFF
        //if (fpsMonitor != null)
        //{
        //    fpsMonitor.Add("width", rgbaMat.width().ToString());
        //    fpsMonitor.Add("height", rgbaMat.height().ToString());
        //    fpsMonitor.Add("orientation", Screen.orientation.ToString());
        //}

        //float width = rgbaMat.width();
        //float height = rgbaMat.height();

        //float widthScale = (float)Screen.width / width;
        //float heightScale = (float)Screen.height / height;
        //if (widthScale < heightScale)
        //{
        //    Camera.main.orthographicSize = (width * (float)Screen.height / (float)Screen.width) / 2;
        //}
        //else
        //{
        //    Camera.main.orthographicSize = height / 2;
        //}
    }


    // Update is called once per frame
    void Update()
    {
        //SelectRoIWithClick();
 
        if (hasInitDone && webCamTexture.isPlaying && webCamTexture.didUpdateThisFrame)
        {
            //Utils.webCamTextureToMat(webCamTexture, rgbaMat, colors);
            Utils.webCamTextureToMat(webCamTexture, greyMat);

            ProcessImgIfRoISelected();

            //Imgproc.putText (rgbaMat, "W:" + rgbaMat.width () + " H:" + rgbaMat.height () + " SO:" + Screen.orientation, new Point (5, rgbaMat.rows () - 10), Imgproc.FONT_HERSHEY_SIMPLEX, 1.0, new Scalar (255, 255, 255, 255), 2, Imgproc.LINE_AA, false);
            //Utils.matToTexture2D(rgbaMat, texture, colors);
            Utils.matToTexture2D(greyMat, texture);
        }
    }

    private void SelectRoIWithClick()
    {
        if (Input.GetMouseButtonDown(0))
        {
            startScreenPos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            startScreenPos.z = Camera.main.nearClipPlane;
        }

        if (Input.GetMouseButtonUp(0))
        {
            endScreenPos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
            endScreenPos.z = Camera.main.nearClipPlane;
            //ConvertScreenPointToTexturePoint(startScreenPos, endScreenPos, webCamTexture.width, webCamTexture.height);
            Point texturePoint = new Point(webCamTexture.width / 2, webCamTexture.height / 2);

            roiStartX = (int)startScreenPos.x + (int)texturePoint.x;
            roiEndX = (int)endScreenPos.x + (int)texturePoint.x;
            roiStartY = (int)-startScreenPos.y + (int)texturePoint.y;
            roiEndY = (int)-endScreenPos.y + (int)texturePoint.y;
            roiWidth = roiEndX - roiStartX;
            roiHeight = roiEndY - roiStartY;
        }
    }

    private void ProcessImgIfRoISelected()
    {
        if (roiEndX > roiStartX && roiEndY > roiStartY)
            roiSelected = true;
        else
        {
            roiSelected = false;
            print("RoI선택 Error(roiStartX > roiEndX & roiStartY > roiEndY)");
        }

        // RoI가 선택됐다면
        if (roiSelected)
        {
            // 스레숄드를 조정해서 Mat에 적용한다.
            if (thresholdMin < thresholdMax)
            {
                //Roi를 잡았으면 그에 따른 ROI Mat, Threshold Mat 생성 -> 원본이미지에 덮어씌우기
                roiMat = new Mat(greyMat, new OpenCVForUnity.CoreModule.Rect(roiStartX, roiStartY, roiWidth, roiHeight));
                newThreshold = new Mat(greyMat, new OpenCVForUnity.CoreModule.Rect(roiStartX, roiStartY, roiWidth, roiHeight));

                // 1. Grayscale to Threshold
                Imgproc.threshold(roiMat, newThreshold, thresholdMin, thresholdMax, Imgproc.THRESH_BINARY/* | Imgproc.THRESH_OTSU*/);

                for (int i = 0; i < newThreshold.rows(); i++)
                {
                    for (int j = 0; j < newThreshold.cols(); j++)
                    {
                        double[] buff = newThreshold.get(i, j);

                        if (buff[0] == 255)
                        {
                            roiCount255++;
                        }
                        else
                            roiCount0++;

                        //if (buff[0] <= 250)
                        //    buff[0] = 0;
                        //else
                        //    buff[0] = 255;

                        //newThreshold.put(i, j, buff);
                    }

                }

                float saturatedRatio = (float)(roiCount255 / (float)(roiCount255 + roiCount0)) * 100;
                //print("Saturated Ratio : " + Math.Truncate(saturatedRatio * 100) / 100 + "% / 255개수 : " + roiCount255 + " / 0개수 : " + roiCount0);
                roiCount255 = roiCount0 = 0;
                // 기존 이미지 위에 newThreshold를 덮어 씌운다.
                greyMat.copyTo(newThreshold);

                // 비율이 70% 이상이면 1초에 한번씩
                if(saturatedRatio > 70.0f)
                {
                    isOpened = true;
                    BluetoothTest.instance.isOpened = true;
                }

                // 물체를 인식하면 문이 열리고, 일정 시간이 되기 전까지 버튼을 활성화
                currentTime += Time.deltaTime;
                if(isOpened && currentTime < openTime)
                {
                    //OnStopButtonClick();
                    luckyButtonUI.SetActive(true);
                    clickIcon.SetActive(true);
                }
                else if(currentTime > openTime)
                {
                    isOpened = false;
                    BluetoothTest.instance.isOpened = false;

                    currentTime = 0;
                    luckyButtonUI.SetActive(false);
                    clickIcon.SetActive(false);
                }


                //// 2. Blur and Canny
                //Size size = new Size(3,3);
                //Imgproc.blur(greyMat, greyMat, size);
                //Imgproc.Canny(thresholdMat, cannyMat, cannyMin, cannyMin * ratio, kernel_size);

                //// 3. Color line
                //// step1. Cany to colorEdge Mat
                //cannyMat.copyTo(colorEdges);
                //Scalar newColor;
                //Imgproc.cvtColor(colorEdges, colorEdges, 1);
                //// step 2. colorEdge Mat의 컬러를 cannyMat에
                //newColor = new Scalar(0, 255, 0);    //this will be green
                //colorEdges.setTo(newColor, cannyMat);
                //// step 3. greyMat 이미지를 cannyMat에
                //colorEdges.copyTo(greyMat, cannyMat);    //this replaces your current cvtColor line, placing your Canny edge lines on the original image

            }

        }
    }
    

    /// <summary>
    /// Raises the destroy event.
    /// </summary>
    void OnDestroy()
    {
        Dispose();
    }

    /// <summary>
    /// Raises the back button click event.
    /// </summary>
    public void OnBackButtonClick()
    {
        SceneManager.LoadScene("OpenCVForUnityExample");
    }

    /// <summary>
    /// Raises the play button click event.
    /// </summary>
    public void OnPlayButtonClick()
    {
        if (hasInitDone)
            webCamTexture.Play();
    }

    /// <summary>
    /// Raises the pause button click event.
    /// </summary>
    public void OnPauseButtonClick()
    {
        if (hasInitDone)
            webCamTexture.Pause();
    }

    /// <summary>
    /// Raises the stop button click event.
    /// </summary>
    public void OnStopButtonClick()
    {
        if (hasInitDone)
            webCamTexture.Stop();
    }

    /// <summary>
    /// Raises the change camera button click event.
    /// </summary>
    public void OnChangeCameraButtonClick()
    {
        if (hasInitDone)
        {
            requestedDeviceName = null;
            requestedIsFrontFacing = !requestedIsFrontFacing;
            Initialize();
        }
    }
}

#endif