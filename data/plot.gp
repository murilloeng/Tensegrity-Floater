#data
if(!exists("index")) index = 6
scale = (index <= 6) ? 1e3 : 1
units = (index <= 6) ? "mm" : "rad"
labels = "x y status u_x u_y u_z r_x r_y r_z"

#setup
unset key
set view map
set size ratio -1
set xlabel "x [m]"
set ylabel "y [m]"
set format x "%.2f"
set format y "%.2f"
set format cb "%.2e"
set pm3d map interpolate 2, 2 corners2color mean
set cblabel sprintf("%s [%s]", word(labels, index), units)
set palette defined (0 "#0000ff", 0.5 "#ffffff", 1 "#ff0000")

#stats
stats file using (sqrt($1**2 + $2**2)) nooutput

#plate
R = STATS_max
set object 1 circle at 0, 0, 0 size R fs empty border lc rgb "black" lw 1.5 front

#plot
splot file using 1 : 2 : (scale * column(index)) with pm3d
