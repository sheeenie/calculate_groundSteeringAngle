# plot.gp - Gnuplot script to visualize steering angle over time

set terminal png size 1024,768
set output 'outputs/steering_plot.png'

set title 'Ground Steering Angle Over Time'
set xlabel 'Time (s)'
set ylabel 'Ground Steering Angle (rad)'
set grid

# Assuming your CSV has headers: timestamp,groundSteeringAngle
set datafile separator ","
set key autotitle columnheader

# Skip the header line using 'every ::1'
plot 'outputs/steering.csv' using 1:2 with lines lw 2 lc rgb 'blue'
