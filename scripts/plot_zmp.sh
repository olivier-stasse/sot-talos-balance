rm -f ~/.config/ros.org/*.ini
touch ~/.config/ros.org/rqt_gui.ini
cat rqt_config.txt >> ~/.config/ros.org/rqt_gui.ini

rqt_plot /sot/dcmCtrl/zmpDes/data[0] /sot/dcmCtrl/zmpRef/data[0] /sot/robot_dynamic/zmp/data[0] /sot/zmpEst/zmp/data[0] &
rqt_plot /sot/dcmCtrl/zmpDes/data[1] /sot/dcmCtrl/zmpRef/data[1] /sot/robot_dynamic/zmp/data[1] /sot/zmpEst/zmp/data[1] &

rqt_plot /sot/fake/comDes/data[0] /sot/comAdmCtrl/comRef/data[0] /sot/robot_dynamic/com/data[0] /sot/cdc_estimator/c/data[0] &
rqt_plot /sot/fake/comDes/data[1] /sot/comAdmCtrl/comRef/data[1] /sot/robot_dynamic/com/data[1] /sot/cdc_estimator/c/data[1] &

