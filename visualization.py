import matplotlib.pyplot as plt
import numpy as np
from jtc_serial import JointData
import matplotlib.style as mplstyle
import time

mplstyle.use('fast')


class JointDataVisualization(JointData):
    fig: plt.Figure
    axes: dict
    lines: dict

    def __init__(self):
        JointData.__init__(self)
        self.fig = plt.Figure()
        self.axes = {}
        self.lines = {}

        self.bm = None

    def create_plot(self):
        self.fig = plt.figure(constrained_layout=True)

        gs = plt.GridSpec(6, 6, figure=self.fig)

        self.axes['pos_ax'] = self.fig.add_subplot(gs[0:2, :3], title='pos')
        self.axes['vel_ax'] = self.fig.add_subplot(gs[0:2, 3:], sharex=self.axes['pos_ax'], title='vel')

        self.axes['tq_ax'] = self.fig.add_subplot(gs[2:4, :], sharex=self.axes['pos_ax'], title='tq')

        self.axes['tq_ID_ax'] = self.fig.add_subplot(gs[4, :3], sharex=self.axes['pos_ax'], title='tq_ID')
        self.axes['tq_fr_ax'] = self.fig.add_subplot(gs[5, :3], sharex=self.axes['pos_ax'], title='tq_fr')

        self.axes['tq_P_ax'] = self.fig.add_subplot(gs[4, 3:], sharex=self.axes['pos_ax'], title='tq_P')

        self.axes['tq_D_ax'] = self.fig.add_subplot(gs[5, 3:], sharex=self.axes['pos_ax'], title='tq_D')

        lines = []
        (self.lines['t_pos_ax'],) = self.axes['pos_ax'].plot(self.time, self.t_pos, color = 'blue')
        (self.lines['c_pos_ax'],) = self.axes['pos_ax'].plot(self.time, self.c_pos, color = 'red')
        (self.lines['t_vel_ax'],) = self.axes['vel_ax'].plot(self.time, self.t_vel,color='blue')
        (self.lines['c_vel_ax'],) = self.axes['vel_ax'].plot(self.time, self.c_vel,color='red')

        # (self.lines['t_tq_ax'],) = self.axes['tq_ax'].plot(self.time, self.t_tq)
        # (self.lines['c_tq_ax'],) = self.axes['tq_ax'].plot(self.time, self.c_tq)
        (self.lines['t_err'],) = self.axes['tq_ax'].plot(self.time, self.t_err)

        (self.lines['tq_ID_ax'],) = self.axes['tq_ID_ax'].plot(self.time, self.tq_ID)
        (self.lines['tq_fr_ax'],) = self.axes['tq_fr_ax'].plot(self.time, self.tq_fr)
        (self.lines['tq_P_ax'],) = self.axes['tq_P_ax'].plot(self.time, self.tq_P)
        (self.lines['tq_D_ax'],) = self.axes['tq_D_ax'].plot(self.time, self.tq_D)

        self.bm = BlitManager(self.fig.canvas,list(self.lines.values()))
        plt.show(block=False)

    def clear_data(self):
        self.t_pos.clear()
        self.t_vel.clear()
        self.t_acc.clear()
        self.t_tq.clear()
        self.tq_P.clear()
        self.tq_I.clear()
        self.tq_D.clear()
        self.tq_PID.clear()
        self.tq_fr.clear()
        self.tq_ID.clear()
        self.c_pos.clear()
        self.c_vel.clear()
        self.c_tq.clear()
        self.c_temp.clear()
        self.t_err.clear()
        self.time.clear()

    def monitor_mode(self):
        self.clear_data()
        while True:
            self.update()
            time.sleep(0.2)

    def update(self):
        self.lines['t_pos_ax'].set_data(self.time, self.t_pos)
        self.lines['c_pos_ax'].set_data(self.time, self.c_pos)
        self.lines['t_vel_ax'].set_data(self.time, self.t_vel)
        self.lines['c_vel_ax'].set_data(self.time, self.c_vel)
        # self.lines['t_tq_ax'].set_data(self.time, self.t_tq)
        # self.lines['c_tq_ax'].set_data(self.time, self.c_tq)
        self.lines['t_err'].set_data(self.time, self.t_err)
        self.lines['tq_ID_ax'].set_data(self.time, self.tq_ID)
        self.lines['tq_fr_ax'].set_data(self.time, self.tq_fr)
        self.lines['tq_P_ax'].set_data(self.time, self.tq_P)
        self.lines['tq_D_ax'].set_data(self.time, self.tq_D)

        for ax in self.axes.values():
            ax.relim()
            ax.autoscale_view()

        self.bm.update()
        # plt.draw()


class BlitManager:
    def __init__(self, canvas, animated_artists=()):
        """
        Parameters
        ----------
        canvas : FigureCanvasAgg
            The canvas to work with, this only works for sub-classes of the Agg
            canvas which have the `~FigureCanvasAgg.copy_from_bbox` and
            `~FigureCanvasAgg.restore_region` methods.

        animated_artists : Iterable[Artist]
            List of the artists to manage
        """
        self.canvas = canvas
        self._bg = None
        self._artists = []

        for a in animated_artists:
            self.add_artist(a)
        # grab the background on every draw
        self.cid = canvas.mpl_connect("draw_event", self.on_draw)

    def on_draw(self, event):
        """Callback to register with 'draw_event'."""
        cv = self.canvas
        if event is not None:
            if event.canvas != cv:
                raise RuntimeError
        self._bg = cv.copy_from_bbox(cv.figure.bbox)
        self._draw_animated()

    def add_artist(self, art):
        """
        Add an artist to be managed.

        Parameters
        ----------
        art : Artist

            The artist to be added.  Will be set to 'animated' (just
            to be safe).  *art* must be in the figure associated with
            the canvas this class is managing.

        """
        if art.figure != self.canvas.figure:
            raise RuntimeError
        art.set_animated(True)
        self._artists.append(art)

    def _draw_animated(self):
        """Draw all of the animated artists."""
        fig = self.canvas.figure
        for a in self._artists:
            fig.draw_artist(a)

    def update(self):
        """Update the screen with animated artists."""
        cv = self.canvas
        fig = cv.figure
        # paranoia in case we missed the draw event,
        if self._bg is None:
            self.on_draw(None)
        else:
            # restore the background
            cv.restore_region(self._bg)
            # draw all of the animated artists
            self._draw_animated()
            # update the GUI state
            cv.blit(fig.bbox)
        # let the GUI event loop process anything it has to do
        cv.flush_events()
