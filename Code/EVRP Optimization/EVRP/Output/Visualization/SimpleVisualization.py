import matplotlib.pyplot as plt
import numpy as np
import pprint
import pandas as pd

if __name__ == "__main__":
    with open("AnalyzedData.txt", "r") as file:
        smalldata = []
        bigdata = []
        for line in file:
            line = line.strip().split("\n")[0].split("\t")
            problem_name = line[0].replace(".txt", "")

            is_big_problem = False
            if("_" in problem_name):
                is_big_problem = True

            problem_data = [problem_name]
            problem_data += [float(line[4]), float(line[7]), float(line[10])]

            if is_big_problem:
                bigdata.append(problem_data)
            else:
                smalldata.append(problem_data)

        pprint.pprint(smalldata)
        pprint.pprint(bigdata)

        columns = ["Problem Instance", "Genetic Algorithm", "Random Search", "NEH with NN Subtours"]
        small_data = pd.DataFrame(smalldata,
                                  columns=columns)

        big_data = pd.DataFrame(bigdata,
                                columns=columns)

        import matplotlib
        matplotlib.style.use('ggplot')

        small_data.plot(x='Problem Instance',
                        kind='bar',
                        stacked=False,
                        title='Solutions of Small Problem Instances by Algorithm',
                        figsize=(10, 5))

        plt.xticks(rotation=30, horizontalalignment="center")
        plt.xlabel("")
        plt.ylabel("Average Distance")
        plt.legend(loc='upper left', ncols=3)
        plt.tight_layout()
        
        plt.savefig('SmallInstanceProblems.pdf', format='pdf', dpi=1200)

        big_data.plot(x='Problem Instance',
                      kind='bar',
                      stacked=False,
                      title='Solutions of Large Problem Instances by Algorithm',
                      figsize=(10, 5))

        plt.xticks(rotation=30, horizontalalignment="center")
        plt.xlabel("")
        plt.ylabel("Average Distance")
        plt.legend(loc='upper left', ncols=3)
        plt.tight_layout()
        plt.savefig('LargeInstanceProblems.pdf', format='pdf', dpi=1200)
