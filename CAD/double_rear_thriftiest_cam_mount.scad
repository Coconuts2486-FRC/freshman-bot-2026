// Converts the camera plate of the supplied dual rear-camera mount for one
// Thriftiest Cam.  Dimensions are in millimeters.
//
// The original plate's local axes are:
//   u: across the plate; v: normal to its flat face; z: vertical.
// Its outside face lies near v = -3.6.  Material farther behind that face is
// the former camera surround and is intentionally removed.

$fn = 48;

// Keep the plate in its original along-plate position.  It is instead moved
// 4 mm normal to the face, into the stand: its full 3.8 mm depth now lies
// within the stand's contact depth.
plate_u_min = 75.0;
plate_u_max = 119.45;
plate_z = 88.9;
plate_thickness = 3.8;
plate_normal_offset = -4.0;
front_v = plate_normal_offset;
rear_v = plate_normal_offset - plate_thickness;

// Thrifty Cam: four 10-32 mounting holes, 1.00 x 1.75 in. pitch.
hole_diameter = 5.2;  // #10 clearance, 0.205 in.
u_center = (plate_u_min + plate_u_max) / 2;
z_center = plate_z / 2;
u_pitch = 25.4;
z_pitch = 44.45;

module in_plate_coordinates() {
    // [u, v, z] -> original STL's [x, y, z]
    multmatrix([
        [0.84,  0.54, 0, 0],
        [0.54, -0.84, 0, 0],
        [0,     0,    1, 0],
        [0,     0,    0, 1]
    ]) children();
}

module new_flat_plate() {
    in_plate_coordinates()
        translate([plate_u_min, rear_v, 0])
            cube([plate_u_max - plate_u_min, plate_thickness, plate_z]);
}

module remove_original_camera_plate() {
    // Remove the old plate and its rear surround in their entirety.  This
    // also removes the former far-end flange.  The new plate is translated
    // into the stand only along its face-normal (v) direction.
    in_plate_coordinates()
        translate([75.0, -35, -0.2])
            cube([60, 70, plate_z + 0.4]);
}

module mounting_holes() {
    in_plate_coordinates()
        for (u = [u_center - u_pitch / 2, u_center + u_pitch / 2])
            for (z = [z_center - z_pitch / 2, z_center + z_pitch / 2])
                translate([u, -10, z])
                    rotate([-90, 0, 0])
                        cylinder(h = 20, d = hole_diameter);
}

difference() {
    union() {
        difference() {
            import("Double Rear camera Mount - original.stl");
            remove_original_camera_plate();
        }
        new_flat_plate();  // fills both former lens openings
    }
    mounting_holes();
}
