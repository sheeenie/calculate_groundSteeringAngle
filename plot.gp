# plot.gp - Gnuplot script to visualize steering angle over time

# Set output format and file
set terminal png size 1024,768 enhanced font 'Arial,12'
set output 'outputs/steering_plot.png'

# Set plot titles and labels
set title 'Ground Steering Angle Over Time' font 'Arial,16'
set xlabel 'Time (seconds)' font 'Arial,14'
set ylabel 'Ground Steering Angle (radians)' font 'Arial,14'

# Enable grid for better readability
set grid xtics ytics

# Set data file format
set datafile separator ","

# Auto-scale and set margins
set autoscale
set lmargin 10
set rmargin 5
set tmargin 5
set bmargin 5

# Plot the data
# Skip header line (row 1) and plot timestamp vs steering angle
plot 'outputs/steering.csv' every ::1 using 1:2 with lines linewidth 2 linecolor rgb 'blue' title 'Steering Angle'

# Print some info
print "Plot saved to outputs/steering_plot.png"