package coordinate.view;

import coordinate.controller.Controller;
import javax.swing.*;

public class Frame  extends JFrame
{
	private Controller app;
	private Panel panel;
	
	public Frame(Controller app)
	{
		super();
		
		this.app = app;
		this.panel = new Panel(this.app);
		setupFrame();
	}

	private void setupFrame()
	{
		this.setTitle("Coordinate Finder");
		this.setSize(800,600);
		this.setContentPane(panel);
		this.setVisible(true);
		this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		this.setResizable(false);
	}
}