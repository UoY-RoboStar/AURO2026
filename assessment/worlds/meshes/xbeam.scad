$rows = 9; // along the y-axis
$cols = 13; // along the x-axis

$m = $cols-1;
$n = $rows-1;

module xbeam() {
    color([0.7,0.7,0.1],0.9)
    translate([0,0,50-2.5]) {
        cube([$cols*5*2+50+15+100, 5, 5], center=true);
    }
}

xbeam();