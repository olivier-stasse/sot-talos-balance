rm -f ~/.config/ros.org/*.ini
touch ~/.config/ros.org/rqt_gui.ini
cat rqt_config.txt >> ~/.config/ros.org/rqt_gui.ini
rqt_plot /sot/base_estimator/q/data[3] /sot/base_link/position/data[3] /sot/base_estimator/q_imu/data[3] &
rqt_plot /sot/base_estimator/q/data[4] /sot/base_link/position/data[4] /sot/base_estimator/q_imu/data[4]&
rqt_plot /sot/base_estimator/q/data[5] /sot/base_link/position/data[5]&
