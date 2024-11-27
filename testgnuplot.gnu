set terminal png size 800,600
set output 'test.png'  # Output to a PNG file for preview
set xlabel "Time [sec]"
set ylabel "Value"
set title "dcR"

plot sin(x) + cos(x)