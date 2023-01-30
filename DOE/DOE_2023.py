import numpy as np
import openmdao.api as om
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, 'C:/Users/demerp3/Desktop/Folder/School/DBF/Year4/Code/DBF_ScoreOP_2023_RPI_PXD')
import ScoreOP_2023 as op

#%% Config

"""
how to save figure:
    rename the created DOEcases.sql file to prevent it from being overwritten
    open case_reader and change cr = om.CaseReader("your_file_name.sql")
"""

sample_size = 1000


#%% Problem Setup
prob = op.prob

# set the driver
driver = prob.driver = om.DOEDriver(om.UniformGenerator(num_samples=sample_size))

# Create a recorder variable
recorder = om.SqliteRecorder('DOEcases.sql')

# Attach a recorder to the problem
prob.add_recorder(recorder)
driver.add_recorder(recorder)

prob.setup()
prob.set_solver_print(level=0)

prob.model.approx_totals()

prob.run_driver()
# prob.record("after_run_driver")

# Instantiate your CaseReader
cr = om.CaseReader("DOEcases.sql")
driver_cases = cr.list_cases('driver')

#%% Initialize lists and append

EP = []
antenna = []
span = []
score = []
con01_vals = []
con02_vals = []
con03_vals = []
con1_vals = []
con2_vals = []
con3_vals = []

for i in range(len(driver_cases)):
    last_case = cr.get_case(driver_cases[i])
    design_vars = last_case.get_design_vars()
    objectives = last_case.get_objectives()
    cons = last_case.get_constraints()
    if design_vars:
        EP.append(design_vars['EP'])
        antenna.append(design_vars['l_a'])
        span.append(design_vars['b'])
        score.append(objectives['obj'])
        con01_vals.append(cons['con01'])
        con02_vals.append(cons['con02'])
        con03_vals.append(cons['con03'])
        con1_vals.append(cons['con1'])
        con2_vals.append(cons['con2'])
        con3_vals.append(cons['con3'])


#%% flip scores

for j in range(len(score)):
    score[j]=score[j]**-1
    #if con01_vals[j]<0.0 or con02_vals[j]<0.0 or con03_vals[j]<0.0 or con1_vals[j]<0.0 or con2_vals[j]<0.0 or con3_vals[j]<0.0:
        #score[j]=np.nan


#%% Below is a short script to see the path the design variables took to convergence
fig, (ax1, ax2, ax3) = plt.subplots(1, 3)
fig.subplots_adjust(left=None, bottom=None, right=None, top=None, wspace=0.5, hspace=None)

ax1.plot(np.arange(len(EP)), np.array(EP))
ax1.set(xlabel='Iterations', ylabel='Design Var: Electronic Package Weight', title='Optimization History')
ax1.grid()

ax2.plot(np.arange(len(antenna)), np.array(antenna))
ax2.set(xlabel='Iterations', ylabel='Design Var: Antenna length', title='Optimization History')
ax2.grid()

ax3.plot(np.arange(len(span)), np.array(span))
ax3.set(xlabel='Iterations', ylabel='Design Var: Wing Span', title='Optimization History')
ax3.grid()


#%% plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

EP_grouped = [round(num[0]/2)*2 for num in EP]
antenna_grouped = [round(num[0]/10)*10 for num in antenna]

p = ax.scatter(EP, antenna_grouped, span, cmap='jet', c=score)
fig.colorbar(p)
ax.set_xlabel('ElecPack Weight (lb)')
ax.set_ylabel('Antenna Length (in)')
ax.set_zlabel('Wing Span (ft)')

ax.view_init(20, 45)

prob.cleanup()