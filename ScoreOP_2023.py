""" module for DBF score optimization (conceptual)"""
import numpy as np
from math import pi
import openmdao.api as om
import AircraftWeight as caw
import TakeoffDistance as dbf

#%% CONFIG & info
""" Aircraft Workflow: Weight -> Lift -> Drag -> Thrust -> repeat
"""

M2 = 2
M3 = 3
Test = 3.5
Score = 4

TestMission = M2


#%% Library

Turnigy_Aerodrive_4240_with_13x8_prop = np.array([130.5, 77.6])/16 # [oz] to [lb] Max Thrust, input cruise thrust


#%% Constants and Constraints
grav = 32.2 # [ft/s2]
rho = 0.00222*grav # [slug/ft3] to [lb/ft3]

motor = Turnigy_Aerodrive_4240_with_13x8_prop
T_max = motor[0] # thrust [lb] @ takeoff; static thrust - MEASURE THIS ASAP

TakeoffDistance = 60 # [ft]
in_Xto = TakeoffDistance
h_o = 7 # wing distance from ground [in]
v0 = 0 # initial velocity; handlaunch vs normal takeoff
cL_max = 1.5 # arbitrary maximum coefficient of lift (elevators/flaps at max deflection)

lap_dist = 3500 # [ft]

#%% Design Parameter Assumptions
in_b = 2.5 # wingspan [ft]
in_c = 0.5 # chordlength [ft]
in_AR = 6.0 # aspect ratio of wing; AR = b^2/S ; S = wing area
in_l_f = 2.5 # fuselage length [ft]
in_ARf = 8.0 # aspect ratio of fuselage; ARf = length/width ; width = height
h_o = 7 # wing height from ground [in]

in_m = 1.0 # motors
in_n = 1.0 # battery packs

in_EP = 3.0 # payload weight [g]
in_l_a = 12.0 # payload antenna length [in]


#%% Output Default Assumptions
in_W = 8.0 # aircraft weight [lb]
in_u = 60.0 # velocity [fps]
in_cd = 0.05 # drag coefficient
in_T = motor[1] # thrust [lb] @ cruise
in_tLap = 70.0 # lap time [s]
in_nLaps = 8.0 # number of laps


#%% Equation - Weight
class Weight(om.ExplicitComponent):
    """ Calculate total weight of the aircraft"""
    
    def setup(self):
        #Inputs
        self.add_input('b', val=in_b)
        self.add_input('c', val=in_c)
        self.add_input('AR', val=in_AR)
        self.add_input('l_f', val=in_l_f)
        self.add_input('ARf', val=in_ARf)
        self.add_input('m', val=in_m)
        self.add_input('n', val=in_n)
        
        self.add_input('EP', val=in_EP)
        self.add_input('l_a', val=in_l_a)

        #Outputs
        self.add_output('W1', val=in_W)
        self.add_output('W2', val=in_W)
        self.add_output('W3', val=in_W)
        

    def setup_partials(self):
        # Finite difference all partials.
        self.declare_partials('*', '*', method='cs')
        

    def compute(self, inputs, outputs):
        """ Sum of the aircraft components """
        b = inputs['b']
        c = inputs['c']
        AR = inputs['AR']
        l_f = inputs['l_f']
        AR_f = inputs['ARf']
        m = inputs['m']
        n = inputs['n']
        
        EP = inputs['EP']
        l_a = inputs['l_a']
        
        the_aircraft = caw.DBFWeight(b, c, AR, l_f, AR_f, m, n)

        outputs['W1'] = the_aircraft.calc_aircraft_mass(0, 0)
        outputs['W2'] = the_aircraft.calc_aircraft_mass(EP, 0)
        outputs['W3'] = the_aircraft.calc_aircraft_mass(0, l_a)


#%% Equation - Drag

class Drag(om.ExplicitComponent):
    """ Calculates the total drag coefficient of the aircraft (parasite + induced)"""

    def setup(self):
        #Inputs
        self.add_input('b', val=in_b)
        self.add_input('AR', val=in_AR)
        self.add_input('l_f', val=in_l_f)
        self.add_input('W1', val=in_W)
        self.add_input('W2', val=in_W)
        self.add_input('W3', val=in_W)
        self.add_input('u1', val=in_u)
        self.add_input('u2', val=in_u)
        self.add_input('u3', val=in_u)
        
        #Outputs
        self.add_output('cd1', val=in_cd)
        self.add_output('cd2', val=in_cd)
        self.add_output('cd3', val=in_cd)


    def setup_partials(self):
        # Finite difference all partials.
        self.declare_partials('*', '*', method='cs')
        

    def compute(self, inputs, outputs):
        """ Evaluates the equation
        cd = 
        """
        b = inputs['b']
        AR = inputs['AR']
        l_f = inputs['l_f']
        W1 = inputs['W1']
        W2 = inputs['W2']
        W3 = inputs['W3']
        u1 = inputs['u1']
        u2 = inputs['u2']
        u3 = inputs['u3']


        outputs['cd1'] = dbf.calc_drag(W1, b, AR, l_f, u1)
        outputs['cd2'] = dbf.calc_drag(W2, b, AR, l_f, u2)
        outputs['cd3'] = dbf.calc_drag(W3, b, AR, l_f, u3)
        
#%% Equation - Velocity

class Velocity(om.ExplicitComponent):
    """ Calculate estimated velocities of the aircraft"""
    
    def setup(self):
        # Inputs
        self.add_input('b', val=in_b)
        self.add_input('AR', val=in_AR)
        self.add_input('cd1', val=in_cd)
        self.add_input('cd2', val=in_cd)
        self.add_input('cd3', val=in_cd)
        self.add_input('T1', val=in_T)
        self.add_input('T2', val=in_T)
        self.add_input('T3', val=in_T)
        
        #Outputs
        self.add_output('u1', val=in_u)
        self.add_output('u2', val=in_u)
        self.add_output('u3', val=in_u)


    def setup_partials(self):
        # Finite difference all partials.
        self.declare_partials('*', '*', method='cs')
        

    def compute(self, inputs, outputs):
        """ Evaluates the equation
        U = sqrt( T / ((1/2) rho cd sw) )
        
        D = (1/2) rho U^2 cd sw
        T = D (cruise)
        solve for U
        """
        b = inputs['b']
        AR = inputs['AR']
        cd1 = inputs['cd1']
        cd2 = inputs['cd2']
        cd3 = inputs['cd3']
        T1 = inputs['T1']
        T2 = inputs['T2']
        T3 = inputs['T3']

        outputs['u1'] = ( T1 / ((1/2)*rho*(b**2/AR)*cd1) )**0.5
        outputs['u2'] = ( T2 / ((1/2)*rho*(b**2/AR)*cd2) )**0.5
        outputs['u3'] = ( T3 / ((1/2)*rho*(b**2/AR)*cd3) )**0.5
        


#%% Equation - Takeoff Distance

class Takeoff(om.ExplicitComponent):
    """
    Calculates the takeoff distance
    """

    def setup(self):

        #Inputs
        self.add_input('b', val=in_b)
        self.add_input('AR', val=in_AR)
        self.add_input('l_f', val=in_l_f)
        self.add_input('W2', val=in_W)
        self.add_input('W3', val=in_W)
        
        #Outputs
        self.add_output('Xto', val=in_Xto)


    def setup_partials(self):
        # Finite difference all partials.
        self.declare_partials('*', '*', method='cs')


    def compute(self, inputs, outputs):
        """
        Evaluates the equation
        nLaps = 600/tLap
        """
        #timeLimit = inputs['timeLimit']
        b = inputs['b']
        AR = inputs['AR']
        l_f = inputs['l_f']
        W2 = inputs['W2']
        W3 = inputs['W3']

        outputs['Xto'] = dbf.takeoff_distance(max(W2,W3), b, AR, l_f, T_max, h_o)


#%% Equation - Time

class Time(om.ExplicitComponent):
    """
    Calculates the time per lap
    """

    def setup(self):
        #Inputs
        self.add_input('u2', val=in_u)
        self.add_input('u3', val=in_u)
        
        #Outputs
        self.add_output('tLap2', val=in_tLap)
        self.add_output('tLap3', val=in_tLap)


    def setup_partials(self):
        # Finite difference all partials.
        self.declare_partials('*', '*', method='cs')
        

    def compute(self, inputs, outputs):
        """
        Evaluates the equation
        tLap = x/u
        """
        u2 = inputs['u2']
        u3 = inputs['u3']

        outputs['tLap2'] = (lap_dist/u2)
        outputs['tLap3'] = (lap_dist/u3)
        
        
#%% Equation - Laps

class Lap(om.ExplicitComponent):
    """ Calculates the number of laps possible
    """

    def setup(self):
        #Inputs
        self.add_input('tLap2', val=in_tLap)
        self.add_input('tLap3', val=in_tLap)
        
        #Outputs
        self.add_output('nLaps2', val=in_nLaps)
        self.add_output('nLaps3', val=in_nLaps)


    def setup_partials(self):
        # Finite difference all partials.
        self.declare_partials('*', '*', method='cs')
        

    def compute(self, inputs, outputs):
        """
        Evaluates the equation
        nLaps2 = 600/tLap2
        """
        #timeLimit = inputs['timeLimit']
        tLap2 = inputs['tLap2']
        tLap3 = inputs['tLap3']


        outputs['nLaps2'] = 600 / tLap2 # seconds
        outputs['nLaps3'] = 300 / tLap3


#%% Combine

class ScoreMDA(om.Group):
    """ Group containing the score MDA.
    """

    def setup(self):
        cycle = self.add_subsystem('cycle', om.Group(), promotes=['*'])
        cycle.add_subsystem('d1', Weight(),
                            promotes_inputs=['b', 'c', 'AR', 'l_f', 'ARf', 'm', 'n', 'EP', 'l_a'],
                            promotes_outputs=['W1', 'W2', 'W3'])
        cycle.add_subsystem('d2', Velocity(),
                            promotes_inputs=['b', 'AR', 'cd1', 'cd2', 'cd3', 'T1', 'T2', 'T3'],
                            promotes_outputs=['u1', 'u2', 'u3'])
        cycle.add_subsystem('d3', Drag(),
                            promotes_inputs=['b', 'AR', 'l_f', 'W1', 'W2', 'W3', 'u1', 'u2', 'u3'],
                            promotes_outputs=['cd1', 'cd2', 'cd3'])
        cycle.add_subsystem('d4', Takeoff(),
                            promotes_inputs=['b', 'AR', 'l_f', 'W2', 'W3'],
                            promotes_outputs=['Xto'])
        cycle.add_subsystem('d5', Time(),
                            promotes_inputs=['u2', 'u3'],
                            promotes_outputs=['tLap2', 'tLap3'])
        cycle.add_subsystem('d6', Lap(),
                            promotes_inputs=['tLap2', 'tLap3'],
                            promotes_outputs=['nLaps2', 'nLaps3'])
        
        
        cycle.set_input_defaults('b', in_b)
        cycle.set_input_defaults('AR', in_AR)
        cycle.set_input_defaults('l_f', in_l_f)
        cycle.set_input_defaults('ARf', in_ARf)
        cycle.set_input_defaults('m', in_m)
        cycle.set_input_defaults('n', in_n)
        cycle.set_input_defaults('EP', in_EP)
        cycle.set_input_defaults('l_a', in_l_a)
        cycle.set_input_defaults('T1', in_T)
        cycle.set_input_defaults('T2', in_T)
        cycle.set_input_defaults('T3', in_T)
        
        # Nonlinear Block Gauss Seidel is a gradient free solver
        cycle.nonlinear_solver = om.NonlinearBlockGS(maxiter=10)
        

# %% Scoring Equations

        #Max Mission 2 Score
        if TestMission == M2:
            self.add_subsystem('obj_cmp', om.ExecComp('obj = ( 1+(EP*nLaps2) )**-1', EP=in_EP, nLaps2=in_nLaps),
                               promotes=['EP', 'nLaps2', 'obj'])

        #Max Mission 3 
        if TestMission == M3:
            self.add_subsystem('obj_cmp', om.ExecComp('obj = ( 2+l_a/tLap3 )**-1', l_a=in_l_a, tLap3=in_tLap),
                               promotes=['l_a', 'obj', 'tLap3'])
            
        #Max Ground Mission Score
        #self.add_subsystem('obj_cmp', on.ExecComp('obj = guhhhh'))
        
        #RPI Score
        if TestMission == Score:
            self.add_subsystem('obj_cmp', om.ExecComp('obj = (1 + 1+(EP*nLaps2)/(10*15) + 2+(l_a/tLap3) / (24/40) )**-1', EP=in_EP, l_a=in_l_a, nLaps2=in_nLaps, tLap3=in_tLap),
                           promotes=['EP', 'l_a', 'nLaps2', 'tLap3', 'obj'])
            

# %% Constraints
        """
        con0X - constraints that are always needed
        con01 - Energy_Battery - Energy_Mission : [V/cell]*[cells] * (3.6[J] / 1[mAh-V])*(2.655[ft-lb] / 3.6[J]) * 1800 [mAh/battery]*[batteries] * prop_efficiency = [lb-ft]
        con02 - Vol_fuselage - Vol_cargo
        con03 - 100s - one_lap
        """

        self.add_subsystem('BatteryConstraint', om.ExecComp('con01 = 3.7*6 * 2.655 * 1800*n * 0.8**m - (nLaps2*T2*lap_dist)'), promotes=['con01', 'T2', 'nLaps2', 'm', 'n']) 
        self.add_subsystem('Cargo_Space', om.ExecComp('con02 = ((l_f**3)/(ARf**2)/2)-(0.25*0.25*0.5)'), promotes=['con02', 'l_f', 'ARf'])
        self.add_subsystem('mission_3_time', om.ExecComp('con03 = 100-tLap3'), promotes=['con03', 'tLap3'])
        #self.add_subsystem('cl1', om.ExecComp('conCL1 = W1 / ((1/2)*rho*u1**2*((b**2)/AR))'), promotes=['conCL1', 'W1', 'u1', 'b', 'AR']) # con = cL
        #self.add_subsystem('cl2', om.ExecComp('conCL2 = W2 / ((1/2)*rho*u2**2*((b**2)/AR))'), promotes=['conCL2', 'W2', 'u2', 'b', 'AR'])
        #self.add_subsystem('cl3', om.ExecComp('conCL3 = W3 / ((1/2)*rho*u3**2*((b**2)/AR))'), promotes=['conCL3', 'W3', 'u3', 'b', 'AR'])
        self.add_subsystem('Takeoff', om.ExecComp('con1 = 60-Xto'), promotes=['con1', 'Xto'])
        self.add_subsystem('Min_Payload', om.ExecComp('con2 = EP - 0.3*W2'), promotes=['con2', 'EP', 'W2'])
        self.add_subsystem('Antenna', om.ExecComp('con3 = (b*12) - l_a'), promotes=['con3', 'l_a', 'b'])

# %% Problem setup

prob = om.Problem()
prob.model = ScoreMDA()

if __name__ == "__main__":
    # set the driver
    
    driver = prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options['optimizer'] = 'SLSQP'
    prob.driver.options['tol'] = 1e-7

    # Create a recorder variable
    recorder = om.SqliteRecorder('OPcases.sql')

    # Attach a recorder to the problem
    prob.add_recorder(recorder)
    driver.add_recorder(recorder)

prob.model.add_design_var('b', lower=2.0, upper=4.0) # wingspan ft
prob.model.add_design_var('c', lower=2/12, upper=6/12) # wingspan ft
prob.model.add_design_var('AR', lower=5.0, upper=7.0)
prob.model.add_design_var('l_f', lower=2.0, upper=4.0) # length fuselage ft
prob.model.add_design_var('ARf', lower=6.0, upper=10.0)
prob.model.add_design_var('m', lower=1.0, upper=1.0) # motor
prob.model.add_design_var('n', lower=1.0, upper=1.0) # batteries
prob.model.add_design_var('T1', lower=0.5, upper=T_max*0.7) # thrust
prob.model.add_design_var('T2', lower=0.5, upper=T_max*0.7) # thrust
prob.model.add_design_var('T3', lower=0.5, upper=T_max*0.7) # thrust
prob.model.add_design_var('EP', lower=3.0, upper=10.0) # Elec Package lb
prob.model.add_design_var('l_a', lower=2.0, upper=24.0) # length antenna in


prob.model.add_constraint('con01', lower=0.0) # Battery
prob.model.add_constraint('con02', lower=0.0) # Cargo
prob.model.add_constraint('con03', lower=0.0) # mission time
#prob.model.add_constraint('conCL1', lower=0.1, upper=0.5) # cl1
#prob.model.add_constraint('conCL2', lower=0.1, upper=0.5) # cl2
#prob.model.add_constraint('conCL3', lower=0.1, upper=0.5) # cl3

prob.model.add_constraint('con1', lower=0.0) # Takeoff
prob.model.add_constraint('con2', lower=0.0) # Payload
prob.model.add_constraint('con3', lower=0.0) # Antenna

prob.model.add_objective('obj')


if __name__ == "__main__":
    # set the driver
    driver = prob.driver = om.ScipyOptimizeDriver()
    prob.driver.options['optimizer'] = 'SLSQP'
    prob.driver.options['tol'] = 1e-7

    # Create a recorder variable
    recorder = om.SqliteRecorder('OPcases.sql')

    # Attach a recorder to the problem
    prob.add_recorder(recorder)
    driver.add_recorder(recorder)
    
    prob.setup()
    prob.set_solver_print(level=0)
    
    prob.model.approx_totals()
    
    prob.cleanup()
    prob.run_driver()
    # prob.record("after_run_driver")
    
    # Instantiate your CaseReader
    cr = om.CaseReader("OPcases.sql")
    driver_cases = cr.list_cases('driver')
    
    #%% Print Results (ctrl+enter to only print results)
    op_b = prob.get_val('b')
    op_AR = prob.get_val('AR')   
    op_S = op_b**2/op_AR
    
    print("GEOMETRY & SPECS")
    print('Wing Span:\t\t\t', op_b, 'ft')
    print('Fuselage Length:\t', prob.get_val('l_f'), 'ft')
    print('Wing aspect Ratio:\t', op_AR)
    print('Fuselage AR:\t\t', prob.get_val('ARf'))
    print('\nMotors:\t\t', prob.get_val('m'))
    print('Batteries:\t', prob.get_val('n'))
    print('Max Cruise Thrust:\t', max(prob.get_val('T1'), prob.get_val('T2'), prob.get_val('T3')), 'lb')
    print('\n-----------------------------------')
    
    print('MISSION 1: Staging Flight\n')
    print('Total Weight:\t', prob.get_val('W1'), 'lb')
    cl1= prob.get_val('W1') / ( (1/2)*rho*(prob.get_val('u1')**2)*op_S)
    print('Lift Coefficient:', cl1)
    print('Drag Coefficient:', prob.get_val('cd1'))
    print('L/D:\t', cl1/prob.get_val('cd1'))
    drag1 = (1/2)*rho*prob.get_val('u1')**2*op_S*prob.get_val('cd1')
    print("\nDrag Force:\t\t", drag1, 'lb')
    print('Cruise Thrust:\t', prob.get_val('T1'), 'lb')
    print('T/W:\t', prob.get_val('T1')/prob.get_val('W1'))
    print('Speed:\t\t\t', prob.get_val('u1'), 'fps')
    print('\n-----------------------------------')
    
    print('MISSION 2: Surveillance Flight\n')
    print('Payload Weight:\t', prob.get_val('EP'), 'lb\n\t\t\t\t', prob.get_val('EP')*453.6, 'g')
    print('Total Weight:\t', prob.get_val('W2'), 'lb')
    cl2= prob.get_val('W2') / ( (1/2)*rho*(prob.get_val('u2')**2)*op_S)
    print('Lift Coefficient:', cl2)
    print('Drag Coefficient:', prob.get_val('cd2'))
    print('L/D:\t', cl2/prob.get_val('cd2'))
    drag2 = (1/2)*rho*prob.get_val('u2')**2*op_S*prob.get_val('cd2')
    print("\nDrag Force:\t\t\t", drag2, 'lb')
    print('Cruise Thrust:\t\t', prob.get_val('T2'), 'lb')
    print('T/W:\t', prob.get_val('T2')/prob.get_val('W2'))
    print('Speed:\t\t\t', prob.get_val('u2'), 'fps')
    print('Number of Laps:\t', prob.get_val('nLaps2'))
    print('\n-----------------------------------')
    
    print('MISSION 3: Jamming Flight\n')
    print('Antenna Length:\t', prob.get_val('l_a'), 'in')
    print('Total Weight:\t', prob.get_val('W3'), 'lb')
    cl3= prob.get_val('W3') / ( (1/2)*rho*(prob.get_val('u3')**2)*op_S)
    print('Lift Coefficient:', cl3)
    print('Drag Coefficient:', prob.get_val('cd3'))
    print('L/D:\t', cl3/prob.get_val('cd3'))
    drag3 = (1/2)*rho*prob.get_val('u3')**2*op_S*prob.get_val('cd3')
    print("\nDrag Force:\t\t\t", drag3, 'lb')
    print('Cruise Thrust:\t', prob.get_val('T3'), 'lb')
    print('T/W:\t', prob.get_val('T3')/prob.get_val('W3'))
    print('Speed:\t\t\t', prob.get_val('u3'), 'fps')
    print('Lap time:\t', prob.get_val('tLap3'), 's')
    print('\n-----------------------------------\n')
    
    score = prob.get_val('obj')**-1
    print('MAX SCORE:\n', score)
    
    prob.cleanup()