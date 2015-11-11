load "format.plt"
set style fill transparent solid 0.5 noborder
unset key 

map_data="map_data.dat"
set size ratio 0.5

set xr [-8:8]
set xtics 2
set mxtics 4
set xlabel "X (m)"

set yr [-4:4]
set ytics 2
set mytics 4
set ylabel "Y (m)"

plot map_data u 2:(-$1):(.10) ev 1 w circles ls 4 notitle ,\
     'radio_1.net' u 2:(-$1):(.10) ev 1 w l ls 1 t 'R1' ,\
     'radio_2.net' u 2:(-$1):(.10) ev 1 w l ls 2 t 'R2' ,\
     'radio_3.net' u 2:(-$1):(.10) ev 1 w l ls 3 t 'R3' ,\
     'radio_4.net' u 2:(-$1):(.10) ev 1 w l ls 1 t 'R4' ,\
     'radio_5.net' u 2:(-$1):(.10) ev 1 w l ls 2 t 'R5' ,\
     'radio_6.net' u 2:(-$1):(.10) ev 1 w l ls 3 t 'R6' ,\
     'radio_7.net' u 2:(-$1):(.10) ev 1 w l ls 1 t 'R7' ,\
     'radio_8.net' u 2:(-$1):(.10) ev 1 w l ls 2 t 'R8' ,\
     'radio_9.net' u 2:(-$1):(.10) ev 1 w l ls 3 t 'R9'
