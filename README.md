# MSCKF

* msckf_mono
  - Modified version of [daniilidis-group/msckf_mono](https://github.com/daniilidis-group/msckf_mono) (Commits d51c9ee on Aug 16, 2018), a **Monocular** MSCKF with ROS Support.
  
* msckf_vio
  - Modified version of [KumarRobotics/msckf_vio](https://github.com/KumarRobotics/msckf_vio) (commit e3a39a9 on Jul 26, 2019), a **Stereo** version of MSCKF.

-----

## Build

```sh
mkdir -p ws_msckf/src
cd ws_msckf/src
git clone xxx
cd ../
catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -j2
```