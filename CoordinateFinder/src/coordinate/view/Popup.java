package coordinate.view;

import javax.swing.JOptionPane;

public class Popup
{
	public void displayMessage(String message)
	{
		JOptionPane.showMessageDialog(null, message);
	}

	public String askQuestion(String question)
	{
		String response = "";
		response = JOptionPane.showInputDialog(null, question);

		if (response == null)
		{
			response = "";
		}
		return response;
	}
}
