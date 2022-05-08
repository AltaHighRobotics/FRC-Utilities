package coordinate.view;

import coordinate.controller.Controller;

import javax.swing.*;
import java.awt.Color;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

public class Panel extends JPanel
{
	private Controller app;
	private SpringLayout layout;
	private JLabel xLabel;
	private JLabel yLabel;

	public Panel(Controller app)
	{
		super();
		this.app = app;
		this.layout = new SpringLayout();
		this.xLabel = new JLabel("x: ");
		this.yLabel = new JLabel("y: ");

		setupPanel();
		setupListeners();
		setupLayout();
	}

	private void setupPanel()
	{

	}

	private void setupLayout()
	{

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
