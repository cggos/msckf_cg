# msckf_vio

-----

[TOC]

# 1. ImageProcessor

## initialize

* `loadParameters`
* create `FastFeatureDetector`

## imuCallback

* `imu_msg_buffer.push_back(*msg)` after the first image

## stereoCallback

* get stereo mono images and timestamps

* `createImagePyramids`
  - `buildOpticalFlowPyramid` for cam0_img and cam1_img

* `initializeFirstFrame`
  - detect `new_features` on the frist cam0_img --> `cam0_points`
  - `stereoMatch`
    - `cam0_points` --> `cam1_points` and `inlier_markers`
  - get `cam0_inliers`, `cam1_inliers` and `response_inliers`
  - Group the features into grids: `grid_new_features`
  - sort by `ImageProcessor::featureCompareByResponse`
  - Collect new features within each grid with high response --> `curr_features_ptr`

* `trackFeatures`
  - `integrateImuData`: compute a rough relative rotation `cam0_R_p_c` and `cam1_R_p_c`
    - 用两帧图像之间的IMU数据，通过积分计算两帧图像的相对旋转矩阵cam0_R_p_c，cam1_R_p_c，直接对角速度进行积分获得旋转矢量，然后用罗德里格斯公式转化成旋转矩阵
  - Organize the features in the previous image: `prev_cam0_points` and `prev_cam1_points`
  - `predictFeatureTracking`
    - `prev_cam0_points` --> `curr_cam0_points`
    - `H = K * cam0_R_p_c * K.inv(), p2 = H * p1`
  - `calcOpticalFlowPyrLK`
    - `prev_cam0_points` --> `curr_cam0_points` and `track_inliers`
  - `stereoMatch`
    - `curr_tracked_cam0_points` --> `curr_cam1_points` and `match_inliers`
  - `twoPointRansac`: remove outliers
    - `prev_matched_cam0_points` and `curr_matched_cam0_points` --> `cam0_ransac_inliers`
    - `prev_matched_cam1_points` and `curr_matched_cam1_points` --> `cam0_ransac_inliers`
  - `curr_matched_cam0_points` and `curr_matched_cam1_points` --> `curr_features_ptr`

* `addNewFeatures`  
  - Create a `mask` to avoid redetecting existing features
  - Detect `new_features` with `mask`
  - Collect the new detected features based on the grid `new_feature_sieve`
  - Select the ones with top response within each grid afterwards and add to `new_features` --> `cam0_points`
  - `stereoMatch`
    - `cam0_points` --> `cam1_points` and `inlier_markers`
  - get `cam0_inliers`, `cam1_inliers` and `response_inliers`
  - Group the features into grids `grid_new_features`
  - sort by `ImageProcessor::featureCompareByResponse`
  - Collect new features within each grid with high response --> `curr_features_ptr`

* `pruneGridFeatures`
  - sort by `ImageProcessor::featureCompareByLifetime`
  - Remove some of the features of a grid by `processor_config.grid_max_feature_num`

* `publish`
  - curr_cam0_points_undistorted[x,y]
  - curr_cam1_points_undistorted[x,y]


### sub func/algms

- `stereoMatch`: `cam0_points` --> `cam1_points` and `inlier_markers`
  - Initialize `cam1_points` by projecting cam0_points to cam1 using the rotation from stereo extrinsics `R_cam0_cam1 = R_cam1_imu.t() * R_cam0_imu`
  - `calcOpticalFlowPyrLK`: `cam0_points` --> `cam1_points` and `inlier_markers`
  - remove outliers: compute the essential matrix `E` --> `inlier_markers`


# 2. MsckfVio

## initialize

* `loadParameters`
* Initialize state server
  - `state_server.continuous_noise_cov`
* Initialize the chi squared test table with confidence level 0.95
  - `chi_squared_test_table`

## imuCallback

* initializeGravityAndBias
  - 将前200帧加速度和角速度求平均
  - `IMUState::gravity`: 平均加速度的模值g作为重力加速度
  - `state_server.imu_state.gyro_bias`: 平均角速度作为陀螺仪的bias
  - `state_server.imu_state.orientation`: 计算重力向量(0,0,-g)和平均加速度之间的夹角(旋转四元数), 标定初始时刻IMU系与world系之间的夹角
  - 因此MSCKF要求前200帧IMU是静止不动的

## featureCallback

* 第一帧时间戳给 `state_server.imu_state.time`

* batchImuProcessing（传播IMU状态，处理两帧图像之间的所有IMU观测数据）
  - processModel（每帧IMU数据）
    - 状态向量预测 predictNewState：姿态预测、速度和位置预测
    - 状态协方差预测
  - `state_server.imu_state.id = IMUState::next_id++;`
  - Remove all used IMU msgs

* stateAugmentation
  - 状态向量扩增
  - 协方差扩增

* addFeatureObservations
  - 将特征添加到map_server, 将特征添加到对应feature.id的observations(std::map)中
  - 计算跟踪已有特征的比例，get `tracking_rate`

* removeLostFeatures（特征跟丢了需要移除特征）
  - Remove the features that lost track from `map_server` and get `jacobian_row_size`
    - `checkMotion`
    - `initializePosition`
  - `featureJacobian`
  - `gatingTest` why
  - `measurementUpdate`
  - Remove all processed features from the map

* pruneCamStateBuffer（相机状态数量达到最大限制`max_cam_state_size`需要剔除掉相机状态）
  - Find two camera states to be removed `findRedundantCamStates`
    - `rotation_threshold`, `translation_threshold`, `tracking_rate_threshold`
    - `sort(rm_cam_state_ids.begin(), rm_cam_state_ids.end());`
  - remove `feature.observations` and get `jacobian_row_size`
    - `checkMotion`
    - `initializePosition`
  - `featureJacobian`
  - `gatingTest` why
  - `measurementUpdate`
  - remove
    - Remove the corresponding rows and columns in the state covariance matrix
    - Remove this camera state in the state vector

* publish
  - odom: `Eigen::Isometry3d T_b_w = IMUState::T_imu_body * T_i_w * IMUState::T_imu_body.inverse();`
  - 3D points: `feature_msg_ptr->points`

### sub func/algms

* `initializePosition`
  - trianglation: `generateInitialGuess`
  - L-M --> position

* `featureJacobian`
  -  Check how many camera states in the provided camera id camera has actually seen this feature
  - `measurementJacobian`
    - (why) Modifty the measurement Jacobian to ensure observability constrain
  - Project the residual and Jacobians onto the nullspace of H_fj
