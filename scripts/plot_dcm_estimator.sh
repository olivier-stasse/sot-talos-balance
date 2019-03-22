rm -f ~/.config/ros.org/*.ini
touch ~/.config/ros.org/rqt_gui.ini
cat rqt_config.txt >> ~/.config/ros.org/rqt_gui.ini

rqt_plot /sot/base_estimator/q/data[0] /sot/base_estimator/q/data[1] /sot/base_estimator/q/data[2] /sot/base_estimator/q/data[3] /sot/base_estimator/q/data[4] /sot/base_estimator/q/data[5] &

rqt_plot /sot/base_estimator/v/data[0] /sot/base_estimator/v/data[1] /sot/base_estimator/v/data[2] /sot/base_estimator/v/data[3] /sot/base_estimator/v/data[4] /sot/base_estimator/v/data[5] &

#rqt_plot /sot/base_estimator/q/data[6] /sot/base_estimator/q/data[7] /sot/base_estimator/q/data[8] /sot/base_estimator/q/data[9] /sot/base_estimator/q/data[10] /sot/base_estimator/q/data[11] /sot/base_estimator/q/data[12] /sot/base_estimator/q/data[13] /sot/base_estimator/q/data[14] /sot/base_estimator/q/data[15] /sot/base_estimator/q/data[16] /sot/base_estimator/q/data[17] /sot/base_estimator/q/data[18] /sot/base_estimator/q/data[19] /sot/base_estimator/q/data[20] /sot/base_estimator/q/data[21] /sot/base_estimator/q/data[22] /sot/base_estimator/q/data[23] /sot/base_estimator/q/data[24] /sot/base_estimator/q/data[25] /sot/base_estimator/q/data[26] /sot/base_estimator/q/data[27] /sot/base_estimator/q/data[28] /sot/base_estimator/q/data[29] /sot/base_estimator/q/data[30] /sot/base_estimator/q/data[31] /sot/base_estimator/q/data[32] /sot/base_estimator/q/data[33] /sot/base_estimator/q/data[34] /sot/base_estimator/q/data[35] /sot/base_estimator/q/data[36] /sot/base_estimator/q/data[37] &

#rqt_plot /sot/base_estimator/v/data[6] /sot/base_estimator/v/data[7] /sot/base_estimator/v/data[8] /sot/base_estimator/v/data[9] /sot/base_estimator/v/data[10] /sot/base_estimator/v/data[11] /sot/base_estimator/v/data[12] /sot/base_estimator/v/data[13] /sot/base_estimator/v/data[14] /sot/base_estimator/v/data[15] /sot/base_estimator/v/data[16] /sot/base_estimator/v/data[17] /sot/base_estimator/v/data[18] /sot/base_estimator/v/data[19] /sot/base_estimator/v/data[20] /sot/base_estimator/v/data[21] /sot/base_estimator/v/data[22] /sot/base_estimator/v/data[23] /sot/base_estimator/v/data[24] /sot/base_estimator/v/data[25] /sot/base_estimator/v/data[26] /sot/base_estimator/v/data[27] /sot/base_estimator/v/data[28] /sot/base_estimator/v/data[29] /sot/base_estimator/v/data[30] /sot/base_estimator/v/data[31] /sot/base_estimator/v/data[32] /sot/base_estimator/v/data[33] /sot/base_estimator/v/data[34] /sot/base_estimator/v/data[35] /sot/base_estimator/v/data[36] /sot/base_estimator/v/data[37] &

rqt_plot /sot/PYRENE/state/data[0] /sot/base_estimator/q/data[0] /sot/base_link/position/data[0] &
rqt_plot /sot/PYRENE/state/data[1] /sot/base_estimator/q/data[1] /sot/base_link/position/data[1] &
rqt_plot /sot/PYRENE/state/data[2] /sot/base_estimator/q/data[2] /sot/base_link/position/data[2] &
#rqt_plot /sot/PYRENE/state/data[3] /sot/base_estimator/q/data[3] /sot/base_link/position/data[3] &
#rqt_plot /sot/PYRENE/state/data[4] /sot/base_estimator/q/data[4] /sot/base_link/position/data[4] &
#rqt_plot /sot/PYRENE/state/data[5] /sot/base_estimator/q/data[5] /sot/base_link/position/data[5] &

rqt_plot /sot/PYRENE/velocity/data[0] /sot/base_estimator/v/data[0] /sot/base_link/velocity/data[0] &
rqt_plot /sot/PYRENE/velocity/data[1] /sot/base_estimator/v/data[1] /sot/base_link/velocity/data[1] &
rqt_plot /sot/PYRENE/velocity/data[2] /sot/base_estimator/v/data[2] /sot/base_link/velocity/data[2] &
#rqt_plot /sot/PYRENE/velocity/data[3] /sot/base_estimator/v/data[3] /sot/base_link/velocity/data[3] &
#rqt_plot /sot/PYRENE/velocity/data[4] /sot/base_estimator/v/data[4] /sot/base_link/velocity/data[4] &
#rqt_plot /sot/PYRENE/velocity/data[5] /sot/base_estimator/v/data[5] /sot/base_link/velocity/data[5] &

rqt_plot /sot/dcm_estimator/c/data[0] /sot/robot_dynamic/com/data[0] &
rqt_plot /sot/dcm_estimator/c/data[1] /sot/robot_dynamic/com/data[1] &
rqt_plot /sot/dcm_estimator/c/data[2] /sot/robot_dynamic/com/data[2] &

rqt_plot /sot/dcm_estimator/dc/data[0] /sot/dcm_estimator/dc/data[1] /sot/dcm_estimator/dc/data[2] &

