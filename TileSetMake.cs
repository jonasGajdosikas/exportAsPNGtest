using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;

namespace saveAsPNGtest
{
    class Program
    {
        static void Main()
        {
            int size = 32;
            int amount = 1;
            for (int i = 1; i <= amount; i++)
            {
                Voronoi map = new Voronoi(size, size, false);
                //Console.WriteLine("Declared map");
                map.Make(i);
                //Console.WriteLine("Saved map");
                Console.WriteLine("{0}/{1}", i, amount);
            }
            /**/
            
            //Console.ReadKey();
        }
    }
    public class Grid
    {
        public int[,] board;
        public int size;
        public int area;
        readonly bool bordered;
        public Random rand;
        public Grid(int _width, int _height, bool _bordered = false)
        {
            size = _width;
            size = _height;
            bordered = _bordered;
            area = size * size;
            board = new int[_width, _height];
            rand = new Random();
        }
        public int this[int x, int y]
        {
            get { return board[x, y]; }
            set { board[x, y] = value; }
        }
        public int this[Coord point]
        {
            get { return this[point.X, point.Y]; }
            set { this[point.X, point.Y] = value; }
        }
        public void Initialize()
        {
            for (int x = 0; x < size; x++)
            {
                for (int y = 0; y < size; y++)
                {
                    this[x, y] = 1;
                }
            }
        }
        public void ConnectStraight(Coord from, Coord to, float radius, float dx = 0, float dy = 0, int value = 0)
        {
            List<Coord> line = GetLine(from, to);
            foreach (Coord tile in line) DrawCircle(tile, radius, value, dx, dy);
        }
        public List<Coord> GetLine(Coord from, Coord to)
        {
            List<Coord> line = new List<Coord>();
            int x = from.X;
            int y = from.Y;
            int dx = to.X - x;
            int dy = to.Y - y;
            bool steep = false;
            int step = Math.Sign(dx);
            int gStep = Math.Sign(dy);
            int longer = Math.Abs(dx);
            int shorter = Math.Abs(dy);

            if (longer < shorter)
            {
                steep = true;
                step = Math.Sign(dy);
                gStep = Math.Sign(dx);
                longer = Math.Abs(dy);
                shorter = Math.Abs(dx);
            }
            int gAccm = longer / 2;
            for (int i = 0; i < longer; i++)
            {
                line.Add(new Coord(x, y));
                if (steep) { y += step; }
                else { x += step; }
                gAccm += shorter;
                if (gAccm >= longer)
                {
                    if (steep) { x += gStep; }
                    else { y += gStep; }
                    gAccm -= longer;
                }
            }

            return line;
        }
        public void DrawCircle(Coord tile, float R, int value, float dx = 0, float dy = 0)
        {
            int r = (int)Math.Floor(R);
            for (int x = -r; x <= r; x++) for (int y = -r; y <= r; y++)
                {
                    if (Math.Pow(x + dx, 2) + Math.Pow(y + dy, 2) <= Math.Pow(R, 2))
                    {
                        int drawX = tile.X + x;
                        int drawY = tile.Y + y;
                        if (InMap(new Coord(drawX, drawY))) this[drawX, drawY] = value;
                    }
                }
        }
        public Bitmap Graphics(Color prime)
        {
            Bitmap result = new Bitmap(size + (bordered ? 2 : 0), size + (bordered ? 2 : 0));
            int[] bitmapData = new int[result.Height * result.Width];
            Dictionary<int, Color> colorDict = new Dictionary<int, Color>
            {
                { -2, Color.White },
                { -1, Color.Black },
                { 0, prime }
            };
            for (int x = 0; x < result.Width; x++)
            {
                for (int y = 0; y < result.Height; y++)
                {
                    Color c;
                    {
                        int val = this[x, y];
                        if (!colorDict.ContainsKey(val))
                        {
                            int R = prime.R * 10 / (10 + val);
                            int G = prime.G * 10 / (10 + val);
                            int B = prime.B * 10 / (10 + val);
                            colorDict.Add(val, Color.FromArgb(R, G, B));
                        }
                        c = colorDict[val];
                    }
                    result.SetPixel(x, y, c);
                    bitmapData[x + y * result.Width] = unchecked((int)0xff000000 | (c.R << 16) | (c.G << 8) | c.B);
                }
            }

            var bits = result.LockBits(new Rectangle(0, 0, result.Width, result.Height), ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
            System.Runtime.InteropServices.Marshal.Copy(bitmapData, 0, bits.Scan0, bitmapData.Length);
            result.UnlockBits(bits);
            return result;
        }
        public bool InMap(Coord coord)
        {
            return (coord >= new Coord(0, 0) && coord < new Coord(size, size));
        }
        public bool InMap(int X, int Y)
        {
            return (X >= 0 && X < size && Y >= 0 && Y < size);
        }
        public int[,] DijkstraDist(List<Coord> from, int startvalue = 0)
        {
            int[,] output = new int[size, size];
            bool[,] reached = new bool[size, size];
            Queue<Coord> queue = new Queue<Coord>();
            for (int x = 0; x < size; x++) for (int y = 0; y < size; y++)
                {
                    output[x, y] = -1;
                    reached[x, y] = false;
                }
            foreach (Coord coord in from)
            {
                queue.Enqueue(coord);
                output[coord.X, coord.Y] = startvalue;
                reached[coord.X, coord.Y] = true;
            }
            while (queue.Count > 0)
            {
                Coord coord = queue.Dequeue();
                foreach (Coord neighbor in coord.Neighbors())
                {
                    if (InMap(neighbor) && !reached[neighbor.X, neighbor.Y] && this[neighbor] != 1)
                    {
                        output[neighbor.X, neighbor.Y] = output[coord.X, coord.Y] + 1;
                        queue.Enqueue(neighbor);
                        reached[neighbor.X, neighbor.Y] = true;
                    }
                }

            }
            return output;
        }
        public int[,] DijkstraDist(Coord from, int startvalue = 0)
        {
            return DijkstraDist(new List<Coord> { from }, startvalue);
        }
        public Bitmap DijkstraGraphics(List<Coord> from)
        {
            int[,] dijkstraMap = DijkstraDist(from);
            Bitmap result = new Bitmap(size + (bordered ? 2 : 0), size + (bordered ? 2 : 0));
            int[] bitmapData = new int[result.Height * result.Width];

            Dictionary<int, Color> colorDict = new Dictionary<int, Color>
            {
                { -1, Color.Black }
            };
            for (int x = 0; x < result.Width; x++)
            {
                for (int y = 0; y < result.Height; y++)
                {
                    Color c;
                    if (bordered)
                    {
                        if (x == 0 || y == 0 || x == result.Width - 1 || y == result.Height - 1)
                        {
                            c = Color.Black;
                        }
                        else
                        {
                            int val = dijkstraMap[x - 1, y - 1];
                            if (!colorDict.ContainsKey(val))
                            {
                                int R = 255;
                                int G = Math.Min(2 * val, 255);
                                int B = Math.Min(2 * val, 255);
                                colorDict.Add(val, Color.FromArgb(R, G, B));
                            }
                            c = colorDict[val];
                        }

                    }
                    else
                    {
                        int val = dijkstraMap[x, y];
                        if (!colorDict.ContainsKey(val))
                        {
                            int R = 255;
                            int G = Math.Min(2 * val, 255);
                            int B = Math.Min(2 * val, 255);
                            colorDict.Add(val, Color.FromArgb(R, G, B));
                        }
                        c = colorDict[val];
                    }
                    result.SetPixel(x, y, c);
                    bitmapData[x + y * result.Width] = unchecked((int)0xff000000 | (c.R << 16) | (c.G << 8) | c.B);
                }
            }

            var bits = result.LockBits(new Rectangle(0, 0, result.Width, result.Height), ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
            System.Runtime.InteropServices.Marshal.Copy(bitmapData, 0, bits.Scan0, bitmapData.Length);
            result.UnlockBits(bits);
            return result;
        }
        public Bitmap DijkstraGraphics(Coord from)
        {
            return DijkstraGraphics(new List<Coord> { from });
        }
    }
    public class Tiling : Grid
    {
        public List<RectRoom> rooms = new List<RectRoom>();
        public List<List<RectRoom>> roomClusters = new List<List<RectRoom>>();
        public Tiling(int _width, int _height, bool _bordered = false) :base (_width, _height, _bordered) { }
       
        public void Erode(int amount, int border = 3)
        {
            for (int i = 0; i < amount; i++)
            {
                Coord pos = new Coord(rand.Next(border, size - 1 - border), rand.Next(border, size - 1 - border));
                int dx = 0;
                int dy = 0;
                int step = 0;
                while (this[pos.X + dx, pos.Y + dy] == 1)
                {
                    step++;
                    pos.X += dx;
                    pos.Y += dy;
                    if (pos.X == size - border) pos.X--;
                    if (pos.Y == size - border) pos.Y--;
                    if (pos.X == border - 1) pos.X++;
                    if (pos.Y == border - 1) pos.Y++;
                    dx = (rand.Next(2) == 0)? ((rand.Next(2) == 0) ? -1 : 1) : 0;
                    dy = (dx==0)? ((rand.Next(2) == 0) ? -1 : 1) : 0;
                }
                this[pos] = 0;
            }
        }
        public void PlaceRoom(int cornerX, int cornerY, int _width, int _height)
        {
            int RCX = rand.Next(cornerX + 3, cornerX + _width - 3);
            int RCY = rand.Next(cornerY + 3, cornerY + _height - 3);
            RectRoom toPlaceRoom = new RectRoom(cornerX, cornerY, RCX, RCY, _width, _height);
            for (int x = toPlaceRoom.corner.X; x < toPlaceRoom.corner.X + toPlaceRoom.width; x++)
            {
                for (int y = toPlaceRoom.corner.Y; y < toPlaceRoom.corner.Y + toPlaceRoom.height; y++)
                {
                    this[x, y] = 0;
                }
            }
            rooms.Add(new RectRoom(cornerX, cornerY, RCX, RCY, _width, _height));
        }
        public void PlaceIrregularRoom(int cX, int cY, int _width, int _height)
        {
            int hB = _width / 4;
            int vB = _height / 4;
            List<Coord> points = new List<Coord>();
            Coord[] corners = new Coord[4] {new Coord(cX + hB, cY), new Coord(cX + 3 * hB, cY + vB),
                                            new Coord(cX + hB, cY + 3*vB), new Coord(cX, cY + vB)};
            int[] widths = new int[4] { 2 * hB, hB, 2 * hB, hB };
            int[] heights = new int[4] { vB, 2 * vB, vB, 2 * vB };

            List<Coord>[] pointsInRect = new List<Coord>[4];
            pointsInRect[0] = new List<Coord>() { new Coord(cX + 1 * hB, cY + 1 * vB)};
            pointsInRect[1] = new List<Coord>() { new Coord(cX + 3 * hB, cY + 1 * vB) };
            pointsInRect[2] = new List<Coord>() { new Coord(cX + 3 * hB, cY + 3 * vB) };
            pointsInRect[3] = new List<Coord>() { new Coord(cX + 1 * hB, cY + 3 * vB) };
            for (int j = 0; j < 4; j++)
            {
                for (int i = 0; i < 4; i++)
                {
                    if (rand.Next(i + 1) > 0) break;
                    int pointX = rand.Next(corners[j].X, corners[j].X + widths[j]);
                    int pointY = rand.Next(corners[j].Y, corners[j].Y + heights[j]);
                    pointsInRect[j].Add(new Coord(pointX, pointY));
                    //Console.WriteLine("placed point #{0} in rectangle #{1}", i+1, j+1);
                }
            }
            pointsInRect[0].Sort((a, b) => a.X.CompareTo(b.X));
            pointsInRect[1].Sort((a, b) => a.Y.CompareTo(b.Y));
            pointsInRect[2].Sort((a, b) => b.X.CompareTo(a.X));
            pointsInRect[3].Sort((a, b) => b.Y.CompareTo(a.Y));
            for (int i = 0; i < 4; i++) points.AddRange(pointsInRect[i]);
            points.Add(pointsInRect[0][0]);
            for (int i = 0; i < points.Count - 1; i++)
            {
                ConnectStraight(points[i], points[i + 1], 1f);
            }
            int RCX = rand.Next(cX + hB, cX + 3 * hB);
            int RCY = rand.Next(cY + vB, cY + 3 * vB);
            rooms.Add(new RectRoom(cX, cY, RCX, RCY, _width, _height));

            //filling the room with empty
            Queue<Coord> queue = new Queue<Coord>();
            queue.Enqueue(new Coord(cX + 2 * hB, cY + 2 * vB));
            this[cX + 2 * hB, cY + 3 * vB] = 0;
            while(queue.Count > 0)
            {
                Coord tile = queue.Dequeue();
                if (tile.X - 1 >= cX) if (this[tile.X - 1, tile.Y] == 1)
                {
                    queue.Enqueue(new Coord(tile.X - 1, tile.Y));
                    this[tile.X - 1, tile.Y] = 0;
                }
                if (tile.Y - 1 >= cY) if (this[tile.X, tile.Y - 1] == 1)
                {
                    queue.Enqueue(new Coord(tile.X, tile.Y - 1));
                    this[tile.X, tile.Y - 1] = 0;
                }
                if(tile.X + 1 < cX + _width) if (this[tile.X + 1, tile.Y] == 1)
                {
                    queue.Enqueue(new Coord(tile.X + 1, tile.Y));
                    this[tile.X + 1, tile.Y] = 0;
                }
                if(tile.Y + 1 < cY + _height) if (this[tile.X, tile.Y + 1] == 1)
                {
                    queue.Enqueue(new Coord(tile.X, tile.Y + 1));
                    this[tile.X, tile.Y + 1] = 0;
                }
            }
        } 
        public float Cosine(Coord A, Coord B)
        {
            return (A.X * B.X + A.Y * B.Y) / (float)Math.Sqrt((A.X ^ 2 + A.Y ^ 2) * (B.X ^ 2 + B.Y ^ 2));
        }
        public void ConnectAllRooms(bool straight = false, float pathWidth = 1.6f, bool Ypaths = false)
        {
            // makes a room cluster for every room
            for (int i = 0; i < rooms.Count; i++)
            {
                List<RectRoom> singleRoom = new List<RectRoom>
                {
                    rooms[i]
                };
                roomClusters.Add(singleRoom);
            }
            // should paths make T-junctions?
            if (Ypaths)
            {
                int amount = rand.Next(1, Math.Max(rooms.Count / 10, 1));
                for (int i = 0; i < amount; i++)
                {
                    int closestConnection = -1;
                    RectRoom roomA = new RectRoom();
                    RectRoom roomB = new RectRoom();
                    RectRoom roomC = new RectRoom();
                    List<RectRoom> clusterA = new List<RectRoom>();
                    List<RectRoom> clusterB = new List<RectRoom>();
                    List<RectRoom> clusterC = new List<RectRoom>();
                    int CAI, CBI, CCI, RAI, RBI, RCI;
                    for (CAI = 0; CAI < roomClusters.Count; CAI++)
                    {
                        for (RAI = 0; RAI < roomClusters[CAI].Count; RAI++)
                        {
                            for (CBI = 0; CBI < roomClusters.Count; CBI++)
                            {
                                if (CBI == CAI) continue;
                                for (RBI = 0; RBI < roomClusters[CBI].Count; RBI++)
                                {
                                    int distance = roomClusters[CAI][RAI].centre.Distance(roomClusters[CBI][RBI].centre);
                                    if (closestConnection == -1 || distance < closestConnection)
                                    {
                                        closestConnection = distance;
                                        roomA = roomClusters[CAI][RAI];
                                        roomB = roomClusters[CBI][RBI];
                                        clusterA = roomClusters[CAI];
                                        clusterB = roomClusters[CBI];
                                    }
                                }
                            }
                        }
                    }
                    roomClusters.Remove(clusterA);
                    roomClusters.Remove(clusterB);
                    clusterA.AddRange(clusterB);
                    List<Coord> line = GetLine(roomA.centre, roomB.centre);
                    int p = 0;
                    while (this[line[p]] == 0) p++;
                    Coord pointA = line[p];
                    p = line.Count - 1;
                    while (this[line[p]] == 0) p--;
                    Coord pointB = line[p];
                    Coord midpoint = pointA.Interpolate(pointB, rand.Next(45, 55));
                    float closestC = -1;
                    Coord AB = roomA.centre - roomB.centre;
                    for (CCI = 0; CCI < roomClusters.Count; CCI++)
                    {
                        for (RCI = 0; RCI < roomClusters[CCI].Count; RCI++)
                        {
                            if (closestC == -1 || midpoint.Dist(roomClusters[CCI][RCI].centre) < closestC) if (Math.Abs(AB & midpoint - roomClusters[CCI][RCI].centre) < 0.5f)
                                {
                                    closestC = midpoint.Dist(roomClusters[CCI][RCI].centre);
                                    roomC = roomClusters[CCI][RCI];
                                    clusterC = roomClusters[CCI];
                                }
                        }
                    }
                    ConnectStraight(roomA.centre, roomB.centre, pathWidth / 2);
                    if (closestC != -1)
                    {
                        ConnectStraight(midpoint, roomC.centre, pathWidth / 2);
                        roomClusters.Remove(clusterC);
                        clusterA.AddRange(clusterC);
                    }
                    roomClusters.Add(clusterA);
                }
            }
            // connects all rooms so there is only one cluster left
            while (roomClusters.Count > 1)
            {
                int closestConnection = -1;
                RectRoom roomA = new RectRoom();
                RectRoom roomB = new RectRoom();
                List<RectRoom> clusterA = new List<RectRoom>();
                List<RectRoom> clusterB = new List<RectRoom>();
                for (int CAI = 0; CAI < roomClusters.Count; CAI++)
                {
                    for (int RAI = 0; RAI < roomClusters[CAI].Count; RAI++)
                    {
                        for (int CBI = 0; CBI < roomClusters.Count; CBI++)
                        {
                            if (CBI == CAI) continue;
                            for (int RBI = 0; RBI < roomClusters[CBI].Count; RBI++)
                            {
                                int distance = roomClusters[CAI][RAI].centre.Distance(roomClusters[CBI][RBI].centre);
                                if (closestConnection == -1 || distance < closestConnection)
                                {
                                    closestConnection = distance;
                                    roomA = roomClusters[CAI][RAI];
                                    roomB = roomClusters[CBI][RBI];
                                    clusterA = roomClusters[CAI];
                                    clusterB = roomClusters[CBI];
                                }
                            }
                        }
                    }
                }
                if (straight) { ConnectStraight(roomA.centre, roomB.centre, pathWidth / 2); }
                else { ConnectRooms(roomA, roomB, pathWidth / 2); }
                roomClusters.Remove(clusterA);
                roomClusters.Remove(clusterB);
                clusterA.AddRange(clusterB);
                roomClusters.Add(clusterA);
            }
        }
        public void ConnectRooms(RectRoom roomA, RectRoom roomB, float radius)
        {
            bool horizontalFirst = (rand.Next(2) == 1);
            Coord midpoint;
            if (horizontalFirst)
            {
                midpoint = new Coord(roomA.centre.X, roomB.centre.Y);
            }
            else
            {
                midpoint = new Coord(roomB.centre.X, roomA.centre.Y);
            }
            ConnectStraight(roomA.centre, midpoint, radius);
            ConnectStraight(roomB.centre, midpoint, radius);
        } 
    }
    public class Voronoi : Tiling
    {
        public int[,] belongsTo;
        public int[,] modifiers;
        public List<Coord> seeds = new List<Coord>();
        public List<VoronoiCell> cells = new List<VoronoiCell>();
        public Voronoi(int _width, int _height, bool _bordered) : base(_width, _height, _bordered) 
        {
            belongsTo = new int[_width, _height];
        }
        public void Make(int i)
        {
            PoissantSample(3);
            Console.WriteLine("Sampled");
            MakeRegions(1);
            Color primeColor = Color.FromArgb(123, 119, 118);
            for (int j = 0; j < 16; j++)
            {
                SetValues(j < 8, j % 8 < 4, j % 4 < 2, j % 2 < 1);
                Filter(Graphics(primeColor), modifiers).Save($"Tiles\\Tile {i} {j}.png");
                Console.WriteLine("Tile {0} out of 32", j + 1);
            }
            for (int j = 0; j < 16; j++)
            {
                SetValuesCorner(j < 8, j % 8 < 4, j % 4 < 2, j % 2 < 1);
                Filter(Graphics(primeColor), modifiers).Save($"Tiles\\Tile {i} {j + 16} corner.png");
                Console.WriteLine("Tile {0} out of 32", j + 17);
            }
            SetValues(false, false, false, false);
            Graphics(primeColor).Save($"Tiles\\Tile {i} 32 full.png");
        }
        public void PoissantSample(float radius, int tries = 15)
        {
            bool[,] hasPoint = new bool[size, size];
            List<Coord> points = new List<Coord>();
            Coord point = new Coord(size/2, size/2);
            hasPoint[point.X, point.Y] = true;
            points.Add(point);
            int failed = 0;
            while (failed < tries + points.Count)
            {
                float dist = radius + (float)rand.NextDouble() * radius;
                float angle = (float)(rand.NextDouble() * Math.PI * 2);
                Coord toTry = point + new Coord(dist, angle);
                bool fits = InMap(toTry);
                int R = (int)Math.Floor(radius);
                for (int x = Math.Max(0, toTry.X - R); x <= Math.Min(size-1, toTry.X + R); x++)
                {
                    for (int y = Math.Max(0, toTry.Y - R); y <= Math.Min(size-1, toTry.Y + R); y++)
                    {
                        if (hasPoint[x,y])
                        {
                            fits &= Math.Pow(x - toTry.X, 2) + Math.Pow(y - toTry.Y, 2) >= Math.Pow(radius, 2);
                            if (!fits) break;
                        }
                    }
                    if (!fits) break;
                }
                if (fits)
                {
                    points.Add(toTry);
                    hasPoint[toTry.X, toTry.Y] = true;
                    //if (failed != 0) Console.WriteLine("failed {0} times out of {1}", failed, tries + points.Count);
                    failed = 0;
                    point = points[rand.Next(points.Count)];
                    //Console.Write(points.Count + " ");
                }
                else
                {
                    failed++;
                    point = points[rand.Next(points.Count)];
                }
            }
            seeds = points;
        }
        public void MakeRegions(int metric = 2)
        {
            for(int i = 0; i < seeds.Count; i++)
            {
                cells.Add(new VoronoiCell(seeds[i], i));
            }
            for (int x = 0; x < size; x++)
            {
                for (int y = 0; y < size; y++)
                {
                    float mindist = -1;
                    int seedIndex = -1;
                    for (int i = 0; i < seeds.Count; i++)
                    {
                        if (mindist == -1 || mindist > seeds[i].Dist(x, y, metric, size))
                        {
                            mindist = seeds[i].Dist(x, y, metric, size);
                            seedIndex = i;
                        }
                    }
                    belongsTo[x, y] = seedIndex;
                    cells[seedIndex].Cells.Add(new Coord(x, y));
                }
            }

        }
        public Bitmap Filter(Bitmap initial, int[,] modifiers)
        {
            Bitmap result = initial;
            int[] bitmapData = new int[result.Height * result.Width];
            for (int x = 0; x < result.Width; x++)
            {
                for (int y = 0; y < result.Height; y++)
                {
                    Color init = initial.GetPixel(x, y);
                    int r = (int)(init.R * (size - modifiers[x, y]) / size);
                    int g = (int)(init.G * (size - modifiers[x, y]) / size);
                    int b = (int)(init.B * (size - modifiers[x, y]) / size);
                    Color c = Color.FromArgb(r, g, b);
                    result.SetPixel(x, y, c);
                    bitmapData[x + y * result.Width] = unchecked((int)0xff000000 | (c.R << 16) | (c.G << 8) | c.B);
                }
            }
            var bits = result.LockBits(new Rectangle(0, 0, result.Width, result.Height), ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
            System.Runtime.InteropServices.Marshal.Copy(bitmapData, 0, bits.Scan0, bitmapData.Length);
            result.UnlockBits(bits);
            return result;
        }
        public void SetValues(bool top, bool right, bool bottom, bool left)
        {
            for (int x = 0; x < size; x++)
            {
                for (int y = 0; y < size; y++)
                {
                    if (belongsTo[x, y] != belongsTo[(x + 1) % size, y] || belongsTo[x, y] != belongsTo[x, (y + 1) % size])
                    {
                        this[x, y] = -1;
                    }
                    else
                    {
                        this[x, y] = (int)seeds[belongsTo[x, y]].Dist(x, y, 2, size);
                    }
                }
            }

            Queue<Coord> queue = new Queue<Coord>();

            if (left)
                for (int y = 0; y < size; y++)
                {
                    queue.Enqueue(new Coord(0, y));
                }
            if (right)
                for (int y = 0; y < size; y++)
                {
                    queue.Enqueue(new Coord(size - 1, y));
                }
            if (top)
                for (int x = 0; x < size; x++)
                {
                    queue.Enqueue(new Coord(x, 0));
                }
            if (bottom)
                for (int x = 0; x < size; x++)
                {
                    queue.Enqueue(new Coord(x, size - 1));
                }

            List<Coord> coords = new List<Coord>();
            coords.AddRange(queue.ToArray());
            modifiers = Gradient(coords);
            while (queue.Count > 0)
            {
                Coord coord = queue.Dequeue();
                if (this[coord] == -1) continue;
                foreach (Coord neighbor in coord.Neighbors())
                {
                    if (InMap(neighbor) && -1 != this[neighbor]) queue.Enqueue(neighbor);
                }
                this[coord] = -1;
            }

        }
        public void SetValuesCorner(bool tr, bool tl, bool br, bool bl)
        {
            for (int x = 0; x < size; x++)
            {
                for (int y = 0; y < size; y++)
                {
                    if (belongsTo[x, y] != belongsTo[(x + 1) % size, y] || belongsTo[x, y] != belongsTo[x, (y + 1) % size])
                    {
                        this[x, y] = -1;
                    }
                    else
                    {
                        this[x, y] = (int)seeds[belongsTo[x, y]].Dist(x, y, 2, size);
                    }
                }
            }
            //Console.WriteLine("set initial values");

            Queue<Coord> queue = new Queue<Coord>();

            if (tl) queue.Enqueue(new Coord(0, 0));
            if (tr) queue.Enqueue(new Coord(size - 1, 0));
            if (bl) queue.Enqueue(new Coord(0, size - 1));
            if (br) queue.Enqueue(new Coord(size - 1, size - 1));

            List<Coord> coords = new List<Coord>();
            coords.AddRange(queue.ToArray());

            //Console.WriteLine("coord list is {0} long", coords.Count);

            modifiers = CircularGradient(coords);

            //Console.WriteLine("set modifiers");

            while (queue.Count > 0)
            {
                Coord coord = queue.Dequeue();
                if (this[coord] == -1) continue;
                foreach (Coord neighbor in coord.Neighbors())
                {
                    if (InMap(neighbor) && -1 != this[neighbor]) queue.Enqueue(neighbor);
                }
                this[coord] = -1;
            }

        }
        public void SetEmpty()
        {
            for (int x = 0; x < size; x++)
            {
                for (int y = 0; y < size; y++)
                {
                    this[x, y] = -1;
                    modifiers[x, y] = 0;
                }
            }

        }
        public int[,] Gradient(List<Coord> from)
        {
            int[,] result = new int[size, size];
            for (int x = 0; x < size; x++) for (int y = 0; y < size; y++) result[x, y] = size - 1;
            Queue<Coord> queue = new Queue<Coord>();
            foreach (Coord coord in from)
            {
                result[coord.X, coord.Y] = 0;
                queue.Enqueue(coord);
            }
            while(queue.Count > 0)
            {
                Coord coord = queue.Dequeue();
                foreach (Coord neighbor in coord.Neighbors())
                {
                    if (InMap(neighbor)) if (result[neighbor.X, neighbor.Y] > result[coord.X, coord.Y] + 1)
                    {
                        result[neighbor.X, neighbor.Y] = result[coord.X, coord.Y] + 1;
                        queue.Enqueue(neighbor);
                    }
                }
            }
            return result;
        }
        public int[,] CircularGradient(List<Coord> from)
        {
            int[,] result = new int[size, size];
            for (int x = 0; x < size; x++) for (int y = 0; y < size; y++)
                {
                    result[x, y] = size - 1; 
                    foreach (Coord point in from) 
                    {
                        result[x, y] = Math.Min(result[x, y], (int)Math.Sqrt(point.Dist(x, y, 2, 0)));
                    }
                }
            return result;
        }
    }
    
    
    //Coord is now basically a vector2, but with int
    public class Coord
    {
        public int X;
        public int Y;
        public Coord(int _x, int _y)
        {
            X = _x;
            Y = _y;
        }
        public Coord(float radius, float angle)
        {
            X = (int)Math.Floor(radius * Math.Cos(angle));
            Y = (int)Math.Floor(radius * Math.Sin(angle));
        }
        public Coord()
        {
            X = 0;
            Y = 0;
        }
        public List<Coord> Neighbors()
        {
            return new List<Coord>
            {
                new Coord(X - 1, Y),
                new Coord(X, Y - 1),
                new Coord(X + 1, Y),
                new Coord(X, Y + 1)
            };
        }
        public int Distance(Coord other)
        {
            return Math.Abs(X - other.X) + Math.Abs(Y - other.Y);
        }
        public float Dist(Coord other, int pow = 2, int size = 16)
        {
            Coord diff = this - other;
            if (size == 0) return (float)(Math.Pow(Math.Abs(diff.X), pow) + Math.Pow(Math.Abs(diff.Y), pow));
            float dist = size * 2;
            for (int x = -size; x <= size; x += size)
            {
                for (int y = -size; y <= size; y += size)
                {
                    dist = Math.Min(dist, (float)(Math.Pow(Math.Abs(diff.X - x), pow) + Math.Pow(Math.Abs(diff.Y - y), pow)));
                }
            }
            return dist;
        }
        public float Dist(int x, int y, int pow = 2, int size = 16)
        {
            return Dist(new Coord(x, y), pow, size);
        }
        public Coord Interpolate(Coord other, int percentage)
        {
            return (percentage * this + (100 - percentage) * other) / 100;
        }
        public static Coord operator +(Coord left, Coord right)
        {
            return new Coord
            {
                X = left.X + right.X,
                Y = left.Y + right.Y
            };
        }
        public static Coord operator -(Coord left, Coord right)
        {
            return new Coord
            {
                X = left.X - right.X,
                Y = left.Y - right.Y
            };
        }
        public static Coord operator *(Coord left, int right)
        {
            return new Coord
            {
                X = left.X * right,
                Y = left.Y * right
            };
        }
        public static Coord operator *(int left, Coord right)
        {
            return new Coord
            {
                X = left * right.X,
                Y = left * right.Y
            };
        }
        public static Coord operator /(Coord left, int right)
        {
            return new Coord
            {
                X = left.X / right,
                Y = left.Y / right
            };
        }
        public static bool operator >(Coord left, Coord right)
        {
            return (left.X > right.X && left.Y > right.Y);
        }
        public static bool operator <(Coord left, Coord right)
        {
            return (left.X < right.X && left.Y < right.Y);
        }
        public static bool operator >=(Coord left, Coord right)
        {
            return (left.X >= right.X && left.Y >= right.Y);
        }
        public static bool operator <=(Coord left, Coord right)
        {
            return (left.X <= right.X && left.Y <= right.Y);
        }
        /*
         * Code for equality based on stored values from
         * https://docs.microsoft.com/en-us/dotnet/csharp/programming-guide/statements-expressions-operators/how-to-define-value-equality-for-a-type
         * Class example. The code there was adapted only with a few changes in variable name
         * 
         */
        public override bool Equals(object obj) => this.Equals(obj as Coord);
        public bool Equals(Coord p)
        {
            if (p is null)
            {
                return false;
            }

            // Optimization for a common success case.
            if (Object.ReferenceEquals(this, p))
            {
                return true;
            }

            // If run-time types are not exactly the same, return false.
            if (this.GetType() != p.GetType())
            {
                return false;
            }

            // Return true if the fields match.
            // Note that the base class is not invoked because it is
            // System.Object, which defines Equals as reference equality.
            return (X == p.X) && (Y == p.Y);
        }
        public override int GetHashCode() => (X, Y).GetHashCode();
        public static bool operator ==(Coord left, Coord right)
        {
            if (left is null)
            {
                if (right is null)
                {
                    return true;
                }

                // Only the left side is null.
                return false;
            }
            // Equals handles case of null on right side.
            return left.Equals(right);
        }
        public static bool operator !=(Coord left, Coord right) => !(left == right);
        public static float operator &(Coord left, Coord right)
        {
            float result = (left.X * right.X + left.Y * right.Y) / (float)Math.Sqrt((left.X * left.X + left.Y * left.Y) * (right.X * right.X + right.Y * right.Y));
            //Console.WriteLine(result);
            return result;
        }
    }
    public class RectRoom
    {
        public Coord corner;
        public Coord centre;
        public int width;
        public int height;
        public RectRoom()
        {
            corner = new Coord(0, 0);
            centre = new Coord(0, 0);
            width = 0;
            height = 0;
        }
        public RectRoom(int cornerX, int cornerY, int centreX, int centreY, int _width, int _height)
        {
            corner = new Coord(cornerX, cornerY);
            centre = new Coord(centreX, centreY);
            width = _width;
            height = _height;
        }
    }
    public class Cell
    {
        public Coord Position;
        public Color colorType;
        public int seed;
        public int dist;
        public Cell(Coord _position)
        {
            Position = _position;
            colorType = new Color();
        }

    }
    public class VoronoiCell
    {
        public Coord Seed;
        public List<Coord> Cells;
        public List<Coord> BorderCells;
        public bool[] isOnEdge;
        public Color color;
        public int index;
        public VoronoiCell(Coord _seed, int _index)
        {
            Seed = _seed;
            Cells = new List<Coord>();
            isOnEdge = new bool[4];
            BorderCells = new List<Coord>();
            index = _index;
        }
    }
}
