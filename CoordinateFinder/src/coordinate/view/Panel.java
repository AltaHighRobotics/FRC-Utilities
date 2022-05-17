package coordinate.view;

import coordinate.controller.Controller;

import javax.swing.*;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

public class Panel extends JPanel
{
	private Controller app;
	private SpringLayout layout;
	private SpringLayout layout_1;
	private JLabel xLabel;
	private JLabel yLabel;
	private ImageIcon playingfield;
	private JLabel imageLabel;

	public Panel(Controller app)
	{
		super();
		this.app = app;
		this.layout = new SpringLayout();
		this.xLabel = new JLabel("x: ");
		this.yLabel = new JLabel("y: ");
		this.playingfield = new ImageIcon(getClass().getResource("/placeholder.png"));
		this.layout_1 = new SpringLayout();
		setupPanel();
		setupListeners();
		setupLayout();
	}

	private void setupPanel()
	{
		this.setPreferredSize(new Dimension(800,600));
		this.add(xLabel);
		this.imageLabel = new JLabel("");
		this.add(imageLabel);
		imageLabel.setIcon(playingfield);
		this.add(yLabel);
		this.setLayout(layout_1);
		
	}

	//319x649
	private void setupLayout()
	{
		layout_1.putConstraint(SpringLayout.NORTH, xLabel, 0, SpringLayout.NORTH, yLabel);
		layout_1.putConstraint(SpringLayout.WEST, xLabel, 78, SpringLayout.WEST, this);
		layout_1.putConstraint(SpringLayout.NORTH, yLabel, 53, SpringLayout.NORTH, this);
		layout_1.putConstraint(SpringLayout.WEST, yLabel, 238, SpringLayout.WEST, this);
		layout_1.putConstraint(SpringLayout.NORTH, imageLabel, 182, SpringLayout.NORTH, this);
		layout_1.putConstraint(SpringLayout.WEST, imageLabel, 202, SpringLayout.WEST, this);
	}

	private void setupListeners()
	{
		this.addMouseListener(new MouseListener()
		{
			public void mousePressed(MouseEvent press)
			{
			}

			public void mouseReleased(MouseEvent release)
			{
			}

			public void mouseEntered(MouseEvent enter)
			{
			}

			public void mouseExited(MouseEvent exit)
			{
			}

			public void mouseClicked(MouseEvent click)
			{
				updatePosition(click);
			}
		});
	}

	private void updatePosition(MouseEvent currentEvent)
	{
		xLabel.setText("x: " + currentEvent.getX());
		yLabel.setText("y: " + currentEvent.getY());
	}

	private void mouseDetails(String name, MouseEvent currentEvent)
	{
		if (currentEvent.getButton() == MouseEvent.BUTTON1)
		{

		}
	}
}
