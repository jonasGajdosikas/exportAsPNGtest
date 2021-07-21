using System;

public class Grid
{
	public int[,] board;
	public int width, height;
	public Grid()
	{
	}
	public Bitmap Draw()
    {
		Bitmap result = new Bitmap(width, height);
    }
}
