""" module for calculating aircraft mass"""
import numpy as np
from math import pi

""" 
NOTES
    calc_fuselage_mass  --> longeron dimensions origin?
    calc_electronics_mass --> flight_control_rod_mass origin?
TO DO year to year
    verify electronics masses
    change payload_mass for mission
    add new material densities to library as necessary
"""
#%% Library

# Densities
carbon_fiber_rho = 1 # [g/cm2] what is this aaaaaaaaaaa
monokote_rho = 0.006 # [g/cm2]
balsa_rho = 0.17 # [g/cm3]
cable_rho = .3651 #g/inch
PVC_rho = 22.9418896 # [g/in3], PVC, =1.4e3kg/m3

# Motors
Turnigy_Aerodrive_4240 = 6.8 # [oz]

#%% Functions
airfoil_data = np.genfromtxt("naca2214.txt")
ft2in2cm = 12*2.54 # [ft] --> [in] --> [cm]

class DBFWeight:
    
    def __init__(self, wingspan, chord, aspect_ratio, length_fuselage, AR_fuselage, num_motor, num_battery):
        """ initialize geometry & weights in FEET, GRAMS, or unitless
        """
        self._wingspan = wingspan
        self._chord = chord
        self._length_fuse = length_fuselage
        self._AR_fuse = AR_fuselage
        self._width_fuse = length_fuselage / AR_fuselage
        self._motor = num_motor
        self._battery = num_battery
        
        
    def calc_fuselage_mass(self):
        """ Fuselage components: skin, bulkheads, longerons
            AR_fuselage = (length/width)_fuselage
        """
        length_fuse = self._length_fuse * ft2in2cm
        width_fuse = self._length_fuse * ft2in2cm
        height_fuse = width_fuse * 1 # square fuselage
        
        ### SKIN
        skin_area = 2*(length_fuse * width_fuse) + 2*(length_fuse * height_fuse)
        skin_mass = skin_area * monokote_rho
        
        ### BULKHEADS
        #num_bulkhead = 8
        bulkhead_per_units_length_fuse = 20 # 1 bulkhead per X units [cm]
        num_bulkhead = 1 + int(np.ceil(length_fuse/bulkhead_per_units_length_fuse)) # endpoints included
        thick_bulk = 0.25 # [cm]
        
        bulkhead_area = (width_fuse*height_fuse) - ((width_fuse-2)*(height_fuse-2))
        bulkhead_mass = bulkhead_area*thick_bulk * balsa_rho * num_bulkhead
        
        ### LONGERON
        longeron_mass = length_fuse * ((0.3175 * 0.3175) - (0.25 * 0.25)) * balsa_rho # (1/8" = 0.3175 cm)
        longeron_mass = 0 # this is an RC aircraft, it won't buckle under its own weight
        
        ### SUM
        fuse_mass = skin_mass + bulkhead_mass + longeron_mass
        return(fuse_mass)
    
    
    def calc_wing_mass(self):
        """ Wing components: ribs, spars, skin
        """
        wingspan = self._wingspan * ft2in2cm
        width_fuse = self._width_fuse * ft2in2cm
        chord = self._chord * ft2in2cm
        airfoil_points = chord * airfoil_data
        
        ### RIBS
        rib_per_units_wingspan = 5 # 1 rib per X units [cm]
        one_wing = (wingspan - width_fuse)/2 # [cm]
        num_ribs = 1 + int(np.ceil(one_wing/rib_per_units_wingspan)) # one wing
        num_ribs *= 2 # two wing
        
        rib_thickness = 0.3175 # [cm] = 1/8"
        #rib_area= ((wingspan*ft2in2cm) / wing_AR) * 4
        rib_area = np.trapz(airfoil_points[:,0], airfoil_points[:,1]) # area of airfoil
        rib_area = 0.7 * rib_area # assume 30% of area is cut out for lightening holes
        rib_mass = rib_area*rib_thickness * balsa_rho * num_ribs
        
        ### SPARS
        spar_per_units_chord = 2.1667 * 2.54 # 1 spar every 2.1667 inches
        num_spars = int(np.ceil(chord/spar_per_units_chord))
        
        max_thicc = max(airfoil_points[:,1])
        spar_area = wingspan * 2 * max_thicc
        #spar_area = wingspan * 2 * trapz(airfoil_points[:,2])/66 #average y value of airfoil
        spar_thickness = 0.3175 # [cm] = 1/8"
        spar_volume = spar_thickness * spar_area# units in cm^3
        spar_mass_unit = spar_volume * balsa_rho #units= g
        spar_mass = spar_mass_unit * num_spars #units = g
        
        ### SKIN
        arc_length = 0 
        for i in range(airfoil_points[:,0].size-1):
            arc_length +=  np.sqrt((airfoil_points[i+1,0]-airfoil_points[i,0])**2 + (airfoil_points[i+1,1]-airfoil_points[i,1])**2)
        skin_surface_area_wing = arc_length * (wingspan - width_fuse) # cm^2
        skin_mass = skin_surface_area_wing * monokote_rho # units =g
        
        ### SUM
        wing_mass = rib_mass + spar_mass + skin_mass
        return(wing_mass)
    
        
    def calc_stabilizer_mass(self):
        """ refine if you dare
        """
        # Assume HT mass is 1/3 of wing mass
        HT_mass = (1/3) * self.calc_wing_mass()
    
        # Assume VT mass is 1/6 of wing mass
        VT_mass = (1/6) * self.calc_wing_mass()
        
        stab_mass = HT_mass + VT_mass
        return(stab_mass)
    
    
    def calc_electronics_mass(self):
        """ units in [grams]
        """
        ### one of each OR num in __init__
        motor_mass = 443 # [g]
        LiPo_battery_mass = 704 # [g]
        propellor_mass = 33 # [g]
        receiver_mass = 14 # [g]
        ESC_mass = 52 # [g]
        
        ### can vary
        servo_motor_mass = 15 # [g] (including 8.5 inch cable)
        servo_num = 5 + 2 # 5 for control surfaces, x for subsystems
        wheel_mass = 20 # [g] (average mass)
        wheel_num = 3 # tricycle
        flight_control_rod_mass = 7 #units=13 inches long
        screws_nuts_mass = 41 # [g] (example total mass of 15 screws and 14 nuts)
        cable_length = 42 # [inches]
        cable_mass = cable_length * cable_rho # [g]
        
        electronics_mass = (motor_mass*self._motor) + (LiPo_battery_mass*self._battery) + propellor_mass + receiver_mass + ESC_mass
        electronics_mass += (servo_motor_mass*servo_num) + (wheel_mass*wheel_num) + flight_control_rod_mass + screws_nuts_mass + cable_mass # [g]
        return(electronics_mass)
    
    
    def calc_payload_mass(self, electronics_package, length_antenna):
        """ 2023: Electronics Package [lb] & Antenna [in]
        """
        d_o = 0.840 # in, 1/2'' 40 schedule
        d_i = 0.602 # in
        A = pi*(d_o/2)**2 - pi*(d_i/2)**2
        vol_antenna = A*(length_antenna) # in3
        mass_antenna = PVC_rho*vol_antenna *2 # [g], *2 for counterweight
        
        payload_mass = mass_antenna + electronics_package/0.00220462
        return payload_mass # end calc_payload_mass
    
    
    def calc_aircraft_mass(self, electronics_package, length_antenna):
        """ sum.
            FS - mass looks too low? fudge it. FS > 1
        """
        FS = 1.1
        total_mass = self.calc_fuselage_mass()
        total_mass += self.calc_wing_mass()
        total_mass += self.calc_stabilizer_mass()
        total_mass += self.calc_electronics_mass()
        total_mass += self.calc_payload_mass(electronics_package, length_antenna)
        total_mass *= 0.00220462 * FS # [g] to [lb]
        return(total_mass)