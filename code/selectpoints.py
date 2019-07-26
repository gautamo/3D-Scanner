import matplotlib.pyplot as plt

class SelectPoints:
    """
    Class that encapsulates allowing the user to click on a matplotlib
    axis and renders the points they clicked on
    """
    def __init__(self,ax,npoints):
        self.ax = ax
        self.pts, = ax.plot([0],[0],'r.')
        self.npoints = npoints
        self.xs = list()
        self.ys = list()
        limits = self.ax.get_xlim()
        self.offset = 0.05*(limits[1]-limits[0])
        print('offset=',self.offset)
        self.cid = self.pts.figure.canvas.mpl_connect('button_press_event',self)

    def __call__(self, event):      
        print('called')
        #ignore clicks outside the figure axis
        if event.inaxes!=self.pts.axes: 
            return

        #otherwise record the click location and draw point on the plot
        self.xs.append(event.xdata)
        self.ys.append(event.ydata)
        self.pts.set_data(self.xs,self.ys)
        self.ax.text(event.xdata+self.offset,event.ydata,('%s'%len(self.xs)),bbox=dict(facecolor='red',alpha=0.3),verticalalignment='top')
        self.pts.figure.canvas.draw()
        
        #once we have npoints clicked, stop listening for
        #click events so that we don't accidentally add more 
        if (len(self.xs) >= self.npoints):
            self.pts.figure.canvas.mpl_disconnect(self.cid)

def select_k_points(ax,npoints):
    """
    Function to allow for interactive selection of points, displaying
    a number along side each point you click in order to make it easier
    to maintain correspondence.
    
    Parameters
    ----------
    ax : matplotlib figure axis
        Indicates the axis where you want to allow the user to select points
    npoints : int
        How many points are needed from the user
        
    Returns
    -------
    SelectPoints object
        Returns an object with fields xs and ys that contain the point 
        coordinates once the user clicks
        
    """

    ax.set_title(('click to select %d points' % npoints))
    selectpoints = SelectPoints(ax,npoints)
    plt.show()
    return selectpoints

