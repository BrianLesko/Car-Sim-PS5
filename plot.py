# Brian Lesko 
# 1/20/2024
# This code support setting Matplotlib plot settings quickly

import numpy as np 
import matplotlib.pyplot as plt

class plotting:

    def __init__(self):
        pass 

    @staticmethod
    def get_colored_plt(hex = "#F6F6F3", hex2 = '#D6D6D6', hex3 = '#D6D6D6',figsize=(6, 6)):
            # hex1 is the background color   # hex2 is the text color    # hex3 is the secondary color
            fig, ax = plt.subplots(figsize=figsize)
            ax.set_facecolor(hex)  
            fig.set_facecolor(hex) 
            for item in ax.get_xticklabels() + ax.get_yticklabels() + ax.get_xgridlines() + ax.get_ygridlines():
                item.set_color(hex2)
            for item in list(ax.spines.values()):
                item.set_color(hex3)
            ax.tick_params(colors=hex3, axis='both', which='major', labelsize=8)
            for spine in ax.spines.values():
                spine.set_linewidth(2)  # Change '2' to your desired linewidth
            return fig, ax
    
    @staticmethod
    def set_axes(ax, xlim=[-2,4], ylim=[-2,2], xlabel='X', ylabel='Y', aspect='equal'):
        ax.set(xlim=xlim, ylim=ylim, xlabel=xlabel, ylabel=ylabel, aspect=aspect)

    @staticmethod
    def get_twin_ax(ax):
        # Set the visibility of the spines to False
        for spine in ax.spines.values():
            spine.set_visible(False)
        # Remove the ticks
        ax.tick_params(axis='both', which='both', length=0)
        ax.set_yticklabels([])
        ax.set_ylim([-2*np.pi,2*np.pi])
        ax.xaxis.label.set_color('#D6D6D6')
        ax.yaxis.label.set_color('#D6D6D6')
        ax2 = ax.twinx()
        return ax2

    def set_c_space_ax(self, ax): 
        pi = np.pi
        self.set_axes(ax,xlim=[-2*pi,2*pi], ylim=[-2*pi,2*pi])
        ax.set_xlabel('$\Theta_1$', color='#D6D6D6')  # Set color of x label
        ax.set_ylabel('$\Theta_2$', color='#D6D6D6')  # Set color of y label
        ax.set_xticks([-2*pi, 0, 2*pi])
        ax.set_yticks([-2*pi, 0, 2*pi])
        ax.set_xticklabels(['-2π', '0', '2π'], color = '#D6D6D6')
        ax.set_yticklabels(['-2π', '0', '2π'], color = '#D6D6D6')
        ax.tick_params(width=0)  # Set the thickness of the tickmarks

    def set_adaptive_ax(self, ax):
        ax.set_xlabel('X', color='#D6D6D6')
        ax.set_ylabel('Y', color='#D6D6D6')
        
        