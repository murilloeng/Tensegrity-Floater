# Top-view heatmap of map_vertical.dat (polar grid, one block per radius)
# usage: gnuplot -e "col=6" map_vertical.gp      (col: 4 ux, 5 uy, 6 uz, 7 rx, 8 ry, 9 rz)
if(!exists("col")) col = 6
scale = (col <= 6) ? 1e3 : 1        # m -> mm for displacements
labels = "x y status u_x u_y u_z r_x r_y r_z"
units = (col <= 6) ? "mm" : "rad"

# set terminal pngcairo size 900,800 enhanced font "Helvetica,14"
# set output sprintf("map-vertical-%s.png", word(labels, col))

set view map
set size ratio -1
set xlabel "x [m]"
set ylabel "y [m]"
set cblabel sprintf("%s [%s]", word(labels, col), units)
set palette defined (0 "#3b4cc0", 0.5 "#f2f2f2", 1 "#b40426")
set pm3d map interpolate 2,2 corners2color mean
unset key

# plate outline
stats "map-vertical.dat" u (sqrt($1**2 + $2**2)) nooutput
R = STATS_max
set object 1 circle at 0,0,0 size R fs empty border lc rgb "black" lw 1.5 front

splot "map-vertical.dat" u 1:2:(column(col) * scale) with pm3d
