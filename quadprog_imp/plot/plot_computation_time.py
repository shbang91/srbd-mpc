import matplotlib.pyplot as plt

qp_prep_dur = []
qp_solve_dur = []
iter_prep = []
iter_solve = []

f_prep = open('experiment_data/qp_prep_dur.txt', 'r')
f_solve = open('experiment_data/qp_solve_dur.txt', 'r')

count = 0
for row in f_prep:
    qp_prep_dur.append(row)
    iter_prep.append(count)
    count += 1

count_solve = 0
for row in f_solve:
    qp_solve_dur.append(row)
    iter_solve.append(count_solve)
    count_solve +=1

fig, axes =plt.subplots(2)
fig.suptitle('mpc compuation time')

axes[0].set_title('qp prep time')
axes[0].plot(iter_prep, qp_prep_dur)

axes[1].set_title('qp solve time')
axes[1].plot(iter_solve, qp_solve_dur)

plt.show()

