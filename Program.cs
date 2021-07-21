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
            
            int mapWidth = 140;
            int mapHeight = 120;
            int amount = 15;
            for (int i = 1; i <= amount; i++)
            {
                VoronoiCaves map = new VoronoiCaves(mapWidth, mapHeight, 20, true);
                //Console.WriteLine("Declared map");
                map.Make();
                map.Graphics().Save($"VoronoiCaves\\Map {i}.png");
                map.DijkstraGraphics(map.rooms[0].Centre).Save($"VoronoiCaves\\Dijkstra {i}.png");
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
        public int gridWidth, gridHeight;
        public int area;
        readonly bool bordered;
        public Random rand;
        public Grid(int _width, int _height, bool _bordered = false)
        {
            gridWidth = _width;
            gridHeight = _height;
            bordered = _bordered;
            area = gridWidth * gridHeight;
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
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
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
        public Bitmap Graphics()
        {
            Bitmap result = new Bitmap(gridWidth + (bordered ? 2 : 0), gridHeight + (bordered ? 2 : 0));
            int[] bitmapData = new int[result.Height * result.Width];
            Dictionary<int, Color> colorDict = new Dictionary<int, Color>
            {
                { 0, Color.White },
                { 1, Color.Black }
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
                            int val = this[x - 1, y - 1];
                            if (!colorDict.ContainsKey(val))
                            {
                                int R = rand.Next(256);
                                int G = rand.Next(256);
                                int B = rand.Next(256);
                                colorDict.Add(val, Color.FromArgb(R, G, B));
                            }
                            c = colorDict[val];
                        }

                    }
                    else
                    {
                        int val = this[x, y];
                        if (!colorDict.ContainsKey(val))
                        {
                            int R = rand.Next(256);
                            int G = rand.Next(256);
                            int B = rand.Next(256);
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
            return (coord >= new Coord(0, 0) && coord < new Coord(gridWidth, gridHeight));
        }
        public bool InMap(int X, int Y)
        {
            return (X >= 0 && X < gridWidth && Y >= 0 && Y < gridHeight);
        }
        public int[,] DijkstraDist(List<Coord> from, int startvalue = 0)
        {
            int[,] output = new int[gridWidth, gridHeight];
            bool[,] reached = new bool[gridWidth, gridHeight];
            Queue<Coord> queue = new Queue<Coord>();
            for (int x = 0; x < gridWidth; x++) for (int y = 0; y < gridHeight; y++)
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
            Bitmap result = new Bitmap(gridWidth + (bordered ? 2 : 0), gridHeight + (bordered ? 2 : 0));
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
                Coord pos = new Coord(rand.Next(border, gridWidth - 1 - border), rand.Next(border, gridHeight - 1 - border));
                int dx = 0;
                int dy = 0;
                int step = 0;
                while (this[pos.X + dx, pos.Y + dy] == 1)
                {
                    step++;
                    pos.X += dx;
                    pos.Y += dy;
                    if (pos.X == gridWidth - border) pos.X--;
                    if (pos.Y == gridHeight - border) pos.Y--;
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
        public List<Coord> seeds = new List<Coord>();
        public List<VoronoiCell> cells = new List<VoronoiCell>();
        public Voronoi(int _width, int _height, bool _bordered) : base(_width, _height, _bordered) 
        {
            belongsTo = new int[_width, _height];
        }
        public void Make()
        {
            PoissantSample(7);
            Console.WriteLine("Sampled");
            MakeRegions();
            MakeRooms();

        }
        public void PoissantSample(float radius, int tries = 15)
        {
            bool[,] hasPoint = new bool[gridWidth, gridHeight];
            List<Coord> points = new List<Coord>();
            Coord point = new Coord(gridWidth/2, gridHeight/2);
            hasPoint[point.X, point.Y] = true;
            points.Add(point);
            int failed = 0;
            while (failed < tries + points.Count)
            {
                if (points.Count > 150) break;
                float dist = radius + (float)rand.NextDouble() * radius;
                float angle = (float)(rand.NextDouble() * Math.PI * 2);
                Coord toTry = point + new Coord(dist, angle);
                bool fits = InMap(toTry);
                int R = (int)Math.Floor(radius);
                for (int x = Math.Max(0, toTry.X - R); x <= Math.Min(gridWidth-1, toTry.X + R); x++)
                {
                    for (int y = Math.Max(0, toTry.Y - R); y <= Math.Min(gridHeight-1, toTry.Y + R); y++)
                    {
                        if (x == toTry.X && y == toTry.Y) continue;
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
                    if (failed != 0) Console.WriteLine("failed {0} times out of {1}", failed, 15 + points.Count);
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
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    float mindist = -1;
                    int seedIndex = -1;
                    for (int i = 0; i < seeds.Count; i++)
                    {
                        if (mindist == -1 || mindist > seeds[i].Dist(x, y, metric))
                        {
                            mindist = seeds[i].Dist(x, y, metric);
                            belongsTo[x, y] = i;
                            seedIndex = i;
                        }
                    }
                    cells[seedIndex].Cells.Add(new Coord(x, y));
                }
            }
            foreach (VoronoiCell cell in cells) cell.CalculateBorders(belongsTo);
        }
        public void MakeRooms(float borderRadius = 1, int distToLoop = 50)
        {
            List<List<VoronoiCell>> clusters = new List<List<VoronoiCell>>();
            // make room in each cell
            foreach (VoronoiCell cell in cells)
            {
                if (cell.isOnEdge) continue;
                //if (rand.Next(100) < 5) continue;
                cell.isRoom = true;
                foreach (Coord coord in cell.Cells)
                {
                    this[coord] = 0;
                }
                foreach (Coord coord in cell.BorderCells)
                {
                    DrawCircle(coord, borderRadius, 1);
                }
                clusters.Add(new List<VoronoiCell> { cell });
            }
            //connect rooms
            while (clusters.Count > 1)
            {
                float bestDist = -1;
                int BCA = -1, BCB = -1, BRA = -1, BRB = -1;
                for (int CAI = 0; CAI < clusters.Count; CAI++)
                {
                    for (int RAI = 0; RAI < clusters[CAI].Count; RAI++)
                    {
                        for (int CBI = 0; CBI < clusters.Count; CBI++)
                        {
                            if (CBI == CAI) continue;
                            for (int RBI = 0; RBI < clusters[CBI].Count; RBI++)
                            {
                                if (bestDist == -1 || bestDist > clusters[CAI][RAI].Seed.Dist(clusters[CBI][RBI].Seed))
                                {
                                    bestDist = clusters[CAI][RAI].Seed.Dist(clusters[CBI][RBI].Seed);
                                    BCA = CAI;
                                    BCB = CBI;
                                    BRA = RAI;
                                    BRB = RBI;
                                }
                            }
                        }
                    }
                }
                List<VoronoiCell> clusterA = clusters[BCA];
                List<VoronoiCell> clusterB = clusters[BCB];
                if (!clusterA[BRA].isRoom || !clusterB[BRB].isRoom) continue;
                clusters.Remove(clusterA);
                clusters.Remove(clusterB);
                clusterA.AddRange(clusterB);
                clusters.Add(clusterA);
                ConnectStraight(clusterA[BRA].Seed, clusterB[BRB].Seed, borderRadius * 1f, 0.5f, 0.5f);
                clusterA[BRA].ConnectedCells.Add(clusterB[BRB].index);
                clusterB[BRB].ConnectedCells.Add(clusterA[BRA].index);

            }
            // make loops if path between adjacent cells is long enough
            foreach (VoronoiCell room in clusters[0])
            {
                int[,] dijkstraMap = DijkstraDist(new List<Coord> { room.Seed });
                int maxdist = 0;
                int maxIndex = -1;
                foreach (int nI in room.AdjacentCells)
                {
                    if (maxdist < dijkstraMap[cells[nI].Seed.X, cells[nI].Seed.Y])
                    {
                        maxdist = dijkstraMap[cells[nI].Seed.X, cells[nI].Seed.Y];
                        maxIndex = nI;
                    }
                }
                if (maxdist >= distToLoop) ConnectStraight(room.Seed, cells[maxIndex].Seed, borderRadius * 1f, 0.5f, 0.5f);
                //Console.WriteLine("checked cell ID{0}; max distance was {1}", room.index, maxdist);
            }
        }
        public void SetValues()
        {
            foreach (VoronoiCell cell in cells)
            {
                foreach (Coord coord in cell.Cells)
                {
                    this[coord] = 0;
                }
                foreach (Coord coord in cell.BorderCells)
                {
                    this[coord] = 1;
                }
            }
        }
    }
    public class VoronoiCaves : Tiling
    {
        public int border;
        public int[,] belongsTo;
        public bool[,] isInner;
        public int[,] mapFlags;
        public List<Coord> seeds = new List<Coord>();
        public List<VoronoiCell> cells = new List<VoronoiCell>();
        new public List<Room> rooms = new List<Room>();
        public VoronoiCaves(int _width, int _height, int _border, bool _bordered) : base(_width, _height, _bordered) 
        {
            belongsTo = new int[_width, _height];
            isInner = new bool[_width, _height];
            border = _border;
        }
        public void Make(int n = 0)
        {
            Initialize();
            //Console.WriteLine("Randomized");
            PoissantSample(11);
            
            MakeRegions(4);
            //Console.WriteLine("Made Regions");
            RandomizeOuter(48);

            MakeRooms();
            //Console.WriteLine("Made Rooms");
            
            SmoothOuter(5);
            //Console.WriteLine("Smoothed");
            RemoveSmall(15);
            /**/
            RemoveInaccessible();
        }
        
        public void PoissantSample(float radius, int tries = 15)
        {
            bool[,] hasPoint = new bool[gridWidth, gridHeight];
            //Console.WriteLine("{0},{1}", gridWidth, gridHeight);
            List<Coord> points = new List<Coord>();
            Coord point = new Coord(gridWidth / 2, gridHeight / 2);
            //Console.WriteLine("{0},{1}", point.X, point.Y);
            this[point] = 0;
            hasPoint[point.X, point.Y] = true;
            points.Add(point);
            int failed = 0;
            while (failed < tries + points.Count)
            {
                float dist = radius + (float)rand.NextDouble() * radius;
                float angle = (float)(rand.NextDouble() * Math.PI * 2);
                Coord toTry = point + new Coord(dist, angle);
                bool fits = InInner(toTry);
                int R = (int)Math.Floor(radius);
                for (int x = toTry.X - R; x <= toTry.X + R; x++)
                {
                    if (!fits) break;
                    for (int y = toTry.Y - R; y <= toTry.Y + R; y++)
                    {
                        if (!fits) break;
                        if (hasPoint[x, y])
                        {
                            fits &= Math.Pow(x - toTry.X, 2) + Math.Pow(y - toTry.Y, 2) >= Math.Pow(radius, 2);
                        }
                    }
                }
                if (fits)
                {
                    //Console.WriteLine("{0},{1}", toTry.X, toTry.Y);
                    points.Add(toTry);
                    //this[toTry] = 0;
                    hasPoint[toTry.X, toTry.Y] = true;
                    failed = 0;
                    point = points[rand.Next(points.Count)];
                    //this.Graphics().Save($"VoronoiCaves\\Point {points.Count}.png");
                    //Console.Write(points.Count + " ");
                }
                else
                {
                    failed++;
                    point = points[rand.Next(points.Count)];
                    //Console.WriteLine("\nfailed " + failed + " times");
                }
            }
            seeds = points;
        }
        public void MakeRegions(int metric = 2)
        {
            for (int i = 0; i < seeds.Count; i++)
            {
                cells.Add(new VoronoiCell(seeds[i], i));
            }
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    float mindist = -1;
                    int seedIndex = -1;
                    for (int i = 0; i < seeds.Count; i++)
                    {
                        if (mindist == -1 || mindist > seeds[i].Dist(x, y, metric))
                        {
                            mindist = seeds[i].Dist(x, y, metric);
                            belongsTo[x, y] = i;
                            seedIndex = i;
                        }
                    }
                    cells[seedIndex].Cells.Add(new Coord(x, y));
                }
            }
            foreach (VoronoiCell cell in cells) cell.CalculateBorders(belongsTo, 20);
        }
        public void MakeRooms(float borderRadius = 1, int distToLoop = 50)
        {
            List<List<VoronoiCell>> clusters = new List<List<VoronoiCell>>();
            // make room in each cell
            foreach (VoronoiCell cell in cells)
            {
                if (cell.isOnEdge) continue;
                //if (rand.Next(100) < 5) continue;
                cell.isRoom = true;
                foreach (Coord coord in cell.Cells)
                {
                    this[coord] = 0;
                }
                foreach (Coord coord in cell.BorderCells)
                {
                    DrawCircle(coord, borderRadius, 1);
                }
                clusters.Add(new List<VoronoiCell> { cell });
            }
            //connect rooms
            while (clusters.Count > 1)
            {
                float bestDist = -1;
                int BCA = -1, BCB = -1, BRA = -1, BRB = -1;
                for (int CAI = 0; CAI < clusters.Count; CAI++)
                {
                    for (int RAI = 0; RAI < clusters[CAI].Count; RAI++)
                    {
                        for (int CBI = 0; CBI < clusters.Count; CBI++)
                        {
                            if (CBI == CAI) continue;
                            for (int RBI = 0; RBI < clusters[CBI].Count; RBI++)
                            {
                                if (bestDist == -1 || bestDist > clusters[CAI][RAI].Seed.Dist(clusters[CBI][RBI].Seed))
                                {
                                    bestDist = clusters[CAI][RAI].Seed.Dist(clusters[CBI][RBI].Seed);
                                    BCA = CAI;
                                    BCB = CBI;
                                    BRA = RAI;
                                    BRB = RBI;
                                }
                            }
                        }
                    }
                }
                List<VoronoiCell> clusterA = clusters[BCA];
                List<VoronoiCell> clusterB = clusters[BCB];
                if (!clusterA[BRA].isRoom || !clusterB[BRB].isRoom) continue;
                clusters.Remove(clusterA);
                clusters.Remove(clusterB);
                clusterA.AddRange(clusterB);
                clusters.Add(clusterA);
                ConnectStraight(clusterA[BRA].Seed, clusterB[BRB].Seed, borderRadius * 1f, 0.5f, 0.5f);
                clusterA[BRA].ConnectedCells.Add(clusterB[BRB].index);
                clusterB[BRB].ConnectedCells.Add(clusterA[BRA].index);

            }
            // make loops if path between adjacent cells is long enough
            foreach (VoronoiCell room in clusters[0])
            {
                int[,] dijkstraMap = DijkstraDist(new List<Coord> { room.Seed });
                int maxdist = 0;
                int maxIndex = -1;
                foreach (int nI in room.AdjacentCells)
                {
                    if (maxdist < dijkstraMap[cells[nI].Seed.X, cells[nI].Seed.Y])
                    {
                        maxdist = dijkstraMap[cells[nI].Seed.X, cells[nI].Seed.Y];
                        maxIndex = nI;
                    }
                }
                if (maxdist >= distToLoop) 
                { 
                    ConnectStraight(room.Seed, cells[maxIndex].Seed, borderRadius * 1f, 0.5f, 0.5f);
                    room.ConnectedCells.Add(maxIndex);
                    cells[maxIndex].ConnectedCells.Add(room.index);
                }
                //Console.WriteLine("checked cell ID{0}; max distance was {1}", room.index, maxdist);
            }
            //make middle room have more connections
            while (cells[0].ConnectedCells.Count < 3){
                int[,] dijkstraMap = DijkstraDist(new List<Coord> { cells[0].Seed });
                int maxdist = 0;
                int maxIndex = -1;
                foreach (int nI in cells[0].AdjacentCells)
                {
                    if (maxdist < dijkstraMap[cells[nI].Seed.X, cells[nI].Seed.Y])
                    {
                        maxdist = dijkstraMap[cells[nI].Seed.X, cells[nI].Seed.Y];
                        maxIndex = nI;
                    }
                }
                ConnectStraight(cells[0].Seed, cells[maxIndex].Seed, borderRadius * 1f, 0.5f, 0.5f);
                cells[0].ConnectedCells.Add(maxIndex);
            }

            Queue<VoronoiCell> queue = new Queue<VoronoiCell>();
            queue.Enqueue(cells[0]);
            cells[0].type = 1;
            while (queue.Count > 0)
            {
                VoronoiCell cell = queue.Dequeue();
                foreach (int conInd in cell.ConnectedCells)
                {
                    if (cells[conInd].type == 0)
                    {
                        cells[conInd].type = cell.type + 1;
                        queue.Enqueue(cells[conInd]);
                    }
                    else
                    {
                        cells[conInd].type = Math.Min(cells[conInd].type, cell.type + 1);
                    }
                }
            }
            //make passages to edge rooms; later caves will form around those
            foreach (VoronoiCell room in clusters[0])
            {
                foreach (Coord cell1 in room.Cells) foreach(Coord cell2 in cell1.Neighbors()) foreach (Coord cell in cell2.Neighbors()) isInner[cell.X, cell.Y] = true;
                foreach (int nI in room.AdjacentCells)
                {
                    if (cells[nI].isOnEdge) ConnectStraight(room.Seed, cells[nI].Seed, borderRadius * 1f, 0.5f, 0.5f);
                }
                Room toAddRoom = new Room(room);
                foreach (Coord cell in toAddRoom.Borders) if (this[cell] == 0) toAddRoom.Doors.Add(cell);
                rooms.Add(new Room(room));
            }
            rooms.Sort();

        }
        public void RandomizeOuter(int infillPercent)
        {
            for (int x = 0; x < gridWidth; x++) for (int y = 0; y < gridHeight; y++) if (!isInner[x, y]) this[x,y] = (rand.Next(100) < infillPercent) ? 0 : 1; 
        }
        public void SmoothOuter(int times)
        {
            for (int i = 0; i < times - 2; i++)
            {
                int[,] neighbors = CountNeigbors();
                for (int x = 0; x < gridWidth; x++)
                {
                    for (int y = 0; y < gridHeight; y++)
                    {
                        if (!isInner[x, y])
                        {
                            if (neighbors[x, y] > 14) this[x, y] = 1;
                            if (neighbors[x, y] < 10) this[x, y] = 0;
                        }
                    }
                }
            }
            for (int i = 0; i < 2; i++)
            {
                int[,] neighbors = CountNeigbors(1);
                for (int x = 0; x < gridWidth; x++)
                {
                    for (int y = 0; y < gridHeight; y++)
                    {
                        if (!isInner[x, y])
                        {
                            if (neighbors[x, y] > 4) this[x, y] = 1;
                            if (neighbors[x, y] < 4) this[x, y] = 0;
                        }
                    }
                }
            }
        }
        public int[,] CountNeigbors(int d = 2)
        {
            int[,] amountNeighbors = new int[gridWidth, gridHeight];
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    amountNeighbors[x, y] = 0;
                    for (int x1 = x - d; x1 <= x + d; x1++)
                    {
                        for (int y1 = y - d; y1 <= y + d; y1++)
                        {
                            if(InMap(x1, y1)){
                                amountNeighbors[x, y] += this[x1, y1];
                            }
                            else
                            {
                                amountNeighbors[x, y]++;
                            }
                        }
                    }
                }
            }
            return amountNeighbors;
        }
        public bool InInner(Coord coord)
        {
            return (coord >= new Coord(border, border) && coord < new Coord(gridWidth - border, gridHeight - border));
        }
        void RemoveSmall(int minSize)
        {
            mapFlags = new int[gridWidth,gridHeight];
            List<List<Coord>> roomRegions = GetRegions(0);
            List<Room> bigRooms = new List<Room>();
            foreach (List<Coord> roomRegion in roomRegions) if (roomRegion.Count < minSize) { foreach (Coord tile in roomRegion) this[tile.X, tile.Y] = 1; }
                else { bigRooms.Add(new Room(roomRegion, board, roomRegion[roomRegion.Count/2])); }
            ConnectClosestRooms(bigRooms);
            rooms.AddRange(bigRooms);
        }

        List<List<Coord>> GetRegions(int tileType)
        {
            List<List<Coord>> regions = new List<List<Coord>>();
            for (int x = 0; x < gridWidth; x++) for (int y = 0; y < gridHeight; y++)
                {
                    if (mapFlags[x, y] == 0 && this[x, y] == tileType)
                    {
                        List<Coord> region = GetRegionTiles(x, y);
                        regions.Add(region);
                    }
                }
            return regions;
        }
        List<Coord> GetRegionTiles(int startX, int startY)
        {
            List<Coord> newRegion = new List<Coord>();
            int tileType = this[startX, startY];

            Queue<Coord> queue = new Queue<Coord>();
            queue.Enqueue(new Coord(startX, startY));
            mapFlags[startX, startY] = 1;
            while (queue.Count > 0)
            {
                Coord tile = queue.Dequeue();
                newRegion.Add(tile);
                if (tile.X > 0 && mapFlags[tile.X - 1, tile.Y] == 0 && this[tile.X - 1, tile.Y] == tileType)
                {
                    queue.Enqueue(new Coord(tile.X - 1, tile.Y));
                    mapFlags[tile.X - 1, tile.Y] = 1;
                }
                if (tile.Y > 0 && mapFlags[tile.X, tile.Y - 1] == 0 && this[tile.X, tile.Y - 1] == tileType)
                {
                    queue.Enqueue(new Coord(tile.X, tile.Y - 1));
                    mapFlags[tile.X, tile.Y - 1] = 1;
                }
                if (tile.X < gridWidth - 1 && mapFlags[tile.X + 1, tile.Y] == 0 && this[tile.X + 1, tile.Y] == tileType)
                {
                    queue.Enqueue(new Coord(tile.X + 1, tile.Y));
                    mapFlags[tile.X + 1, tile.Y] = 1;
                }
                if (tile.Y < gridHeight - 1 && mapFlags[tile.X, tile.Y + 1] == 0 && this[tile.X, tile.Y + 1] == tileType)
                {
                    queue.Enqueue(new Coord(tile.X, tile.Y + 1));
                    mapFlags[tile.X, tile.Y + 1] = 1;
                }
            }
            return newRegion;
        }
        void ConnectClosestRooms(List<Room> allRooms)
        {
            int bestDistance = 0;
            Room bestRoomA;
            Room bestRoomB;
            Coord bestTileA = new Coord();
            Coord bestTileB = new Coord();
            bool connectionFound;
            List<List<Room>> roomClusters = new List<List<Room>>();
            List<Room> bestClusterA = new List<Room>();
            List<Room> bestClusterB = new List<Room>();
            foreach (Room Room in allRooms)
            {
                List<Room> cluster = new List<Room> {Room };
                roomClusters.Add(cluster);
            }
            while (roomClusters.Count > 1)
            {
                connectionFound = false;
                foreach (List<Room> clusterA in roomClusters)
                {
                    foreach (List<Room> clusterB in roomClusters)
                    {
                        if (clusterA == clusterB) continue;
                        foreach (Room roomA in clusterA)
                        {
                            foreach (Room roomB in clusterB)
                            {
                                for (int i = 0; i < roomA.Borders.Count; i++)
                                {
                                    for (int j = 0; j < roomB.Borders.Count; j++)
                                    {
                                        Coord tileA = roomA.Borders[i];
                                        Coord tileB = roomB.Borders[j];
                                        int distanceBetweenRooms = (int)tileA.Dist(tileB);
                                        if (distanceBetweenRooms <= bestDistance || !connectionFound)
                                        {
                                            bestClusterA = clusterA;
                                            bestClusterB = clusterB;
                                            bestRoomA = roomA;
                                            bestRoomB = roomB;
                                            bestTileA = tileA;
                                            bestTileB = tileB;
                                            bestDistance = distanceBetweenRooms;
                                            connectionFound = true;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if (connectionFound)
                {
                    roomClusters.Remove(bestClusterA);
                    roomClusters.Remove(bestClusterB);
                    bestClusterA.AddRange(bestClusterB);
                    roomClusters.Add(bestClusterA);
                    ConnectStraight(bestTileA, bestTileB, 2f);
                }
            }
        }
        void RemoveInaccessible()
        {
            int[,] dijkstra = DijkstraDist(cells[0].Seed);
            for (int x = 0; x < gridWidth; x++)
            {
                for (int y = 0; y < gridHeight; y++)
                {
                    if (dijkstra[x, y] == -1 && this[x, y] == 0) this[x, y] = 1;
                }
            }
        }
    }
    public class PlacedRooms : Tiling
    {
        public PlacedRooms(int _width, int _height, bool _bordered) : base(_width, _height, _bordered) { }

        
    }
    public class BSP : Tiling
    {
        public BSP(int _width, int _height, bool _bordered) : base(_width, _height, _bordered) { }
        public List<Partition> partitions = new List<Partition>();
        
        public void GeneratePartitions(int steps, int areaPart, int biggerPartitionPercentage = 0)
        {
            partitions.Add(new Partition(0, 0, gridWidth, gridHeight));
            int n = 1;
            for (int i = 0; i < steps; i++)
            {
                for (int j = n - 1; j < 2 * n - 1; j++){
                    if (rand.Next(100) < biggerPartitionPercentage && i == steps-1) continue;
                    int Hor = rand.Next(2);
                    if (4 * partitions[j].width > 5 * partitions[j].height) Hor = 0;
                    if (5 * partitions[j].width < 4 * partitions[j].height) Hor = 1;
                    int position = rand.Next(areaPart, 101 - areaPart);
                    int x1 = partitions[j].Corner.X;
                    int y1 = partitions[j].Corner.Y;
                    int x2, y2;
                    int w1, h1, w2, h2;
                    if (Hor == 1)
                    {
                        w1 = partitions[j].width;
                        w2 = partitions[j].width;
                        h1 = position * partitions[j].height / 100;
                        h2 = partitions[j].height - h1;
                        x2 = x1;
                        y2 = y1 + h1;
                    }
                    else
                    {
                        w1 = position * partitions[j].width / 100;
                        w2 = partitions[j].width - w1;
                        h1 = partitions[j].height;
                        h2 = partitions[j].height;
                        x2 = x1 + w1;
                        y2 = y1;
                    }
                    partitions[j].small = false;
                    partitions.Add(new Partition(x1, y1, w1, h1));
                    partitions.Add(new Partition(x2, y2, w2, h2));
                }
                n *= 2;
            }
        }

        public void AddRoomsInPartitions(int _border, bool irregular = false, int chanceToMiss = 0)
        {
            for (int i = 0; i < partitions.Count; i++)
            {
                if (!partitions[i].small) continue;
                if (rand.Next(100) < chanceToMiss) continue;
                int roomW;
                int roomH;
                int roomX;
                int roomY;
                if (!irregular)
                {
                    roomW = rand.Next(4, partitions[i].width / 2 - _border) + rand.Next(4, partitions[i].width / 2 - _border);
                    roomH = rand.Next(4, partitions[i].height / 2 - _border) + rand.Next(4, partitions[i].height / 2 - _border);
                    roomX = rand.Next(partitions[i].Corner.X + _border, partitions[i].Corner.X + partitions[i].width - roomW - _border);
                    roomY = rand.Next(partitions[i].Corner.Y + _border, partitions[i].Corner.Y + partitions[i].height - roomH - _border);
                    PlaceRoom(roomX, roomY, roomW, roomH);
                }
                else
                {
                    _border = rand.Next(2*_border / 3,5*_border/3);
                    roomW = partitions[i].width - 2 * _border;
                    roomH = partitions[i].height - 2 * _border;
                    roomX = partitions[i].Corner.X + _border;
                    roomY = partitions[i].Corner.Y + _border;
                    PlaceIrregularRoom(roomX, roomY, roomW, roomH);
                }
            }
        }
        public int SP(int percentage)
        {
            return Math.Min(percentage, 100 - percentage);
        }
    }
    public class Maze : Tiling
    {
        public Maze(int _width, int _height, bool _bordered) : base(_width, _height, _bordered) { }
        public void Setup()
        {
            for (int x = 0; x < gridWidth; x += 2)
            {
                for (int y = 0; y < gridHeight; y += 2)
                {
                    this[x, y] = 0;
                    rooms.Add(new RectRoom(x, y, x, y, 1, 1));
                }
            }
        }
        public void Connect()
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
            
            float pathWidth = 0.6f;
            // connects all rooms so there is only one cluster left
            while (roomClusters.Count > 1)
            {
                int closestConnection = -1;
                List<RectRoom> roomAs = new List<RectRoom>();
                List<RectRoom> roomBs = new List<RectRoom>();
                List<List<RectRoom>> clusterAs = new List<List<RectRoom>>();
                List<List<RectRoom>> clusterBs = new List<List<RectRoom>>();
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
                                    roomAs.Clear();
                                    roomBs.Clear();
                                    clusterAs.Clear();
                                    clusterBs.Clear();
                                }
                                if (closestConnection == distance)
                                {
                                    roomAs.Add(roomClusters[CAI][RAI]);
                                    roomBs.Add(roomClusters[CBI][RBI]);
                                    clusterAs.Add(roomClusters[CAI]);
                                    clusterBs.Add(roomClusters[CBI]);
                                }
                            }
                        }
                    }
                }
                int connection = rand.Next(roomAs.Count);
                ConnectRooms(roomAs[connection], roomBs[connection], pathWidth / 2);
                roomClusters.Remove(clusterAs[connection]);
                roomClusters.Remove(clusterBs[connection]);
                clusterAs[connection].AddRange(clusterBs[connection]);
                roomClusters.Add(clusterAs[connection]);
                
            }
        }
    }
    /*Here begin nonmap classes */
    public class Partition
    {
        public Coord Corner;
        public int width;
        public int height;
        public bool small;

        public Partition(int X, int Y, int _width, int _height)
        {
            Corner = new Coord(X, Y);
            width = _width;
            height = _height;
            small = true;
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
        public float Dist(Coord other, int pow = 2)
        {
            Coord diff = this - other;
            return (float)(Math.Pow(diff.X,pow) + Math.Pow(diff.Y, pow));
        }
        public float Dist(int x, int y, int pow = 2)
        {
            Coord diff = this - new Coord(x,y);
            return (float)(Math.Pow(diff.X, pow) + Math.Pow(diff.Y, pow));
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
    public class Room : IComparable<Room>
    {
        public List<Coord> InnerCells;
        public List<Coord> Borders;
        public List<Coord> Doors;
        public Coord Centre;
        public int roomSize;
        public int index;
        public int type;
        public List<int> ConnectedRooms;
        public Room()
        {
            InnerCells = new List<Coord>();
            Borders = new List<Coord>();
            Doors = new List<Coord>();
            Centre = new Coord();
            ConnectedRooms = new List<int>();
        }
        public Room(VoronoiCell voronoi)
        {
            InnerCells = new List<Coord>();
            Borders = voronoi.BorderCells;
            Doors = new List<Coord>();
            Centre = voronoi.Seed;
            foreach (Coord cell in voronoi.Cells)
            {
                if (!Borders.Contains(cell)) InnerCells.Add(cell);
            }
            roomSize = InnerCells.Count;
            ConnectedRooms = voronoi.ConnectedCells;
            index = voronoi.index;
            type = voronoi.type;
        }
        public Room(List<Coord> _cells, int[,] map, Coord _centre, int _type = -1)
        {
            Coord corner1 = new Coord(0, 0);
            Coord corner2 = new Coord(map.GetLength(0), map.GetLength(1));
            InnerCells = _cells;
            Centre = _centre;
            Borders = new List<Coord>();
            foreach(Coord inner in InnerCells)
            {
                foreach(Coord test in inner.Neighbors())
                {
                    if (!(test >= corner1 && test < corner2)) continue;
                    if (map[test.X, test.Y] == 1)
                    {
                        if (!Borders.Contains(test)) Borders.Add(test);
                    }
                }
            }
            index = -1;
            type = _type;
        }
        public int CompareTo(Room other)
        {
            if (index == -1)
            {
                if (other.index == -1) return 0;
                return 1;
            }
            if (other.index == -1) return -1;
            return index.CompareTo(other.index);
        }
    }
    
    public class VoronoiCell
    {
        public Coord Seed;
        public List<Coord> Cells;
        public List<Coord> BorderCells;
        public bool isRoom;
        public bool isOnEdge;
        public List<int> AdjacentCells;
        public List<int> ConnectedCells;
        public int index;
        public int type;
        public VoronoiCell(Coord _seed, int _index)
        {
            Seed = _seed;
            Cells = new List<Coord>();
            isRoom = false;
            isOnEdge = false;
            AdjacentCells = new List<int>();
            BorderCells = new List<Coord>();
            ConnectedCells = new List<int>();
            index = _index;
            type = 0;
        }
        public void CalculateBorders(int[,] map, int border = 0)
        {
            foreach (Coord point in Cells)
            {
                bool isBorder = false;
                int x = point.X;
                int y = point.Y;
                if (x <= border) isOnEdge = true;
                if (x > border && map[x, y] != map[x - 1, y])
                {
                    isBorder = true;
                    if (!AdjacentCells.Contains(map[x - 1, y])) AdjacentCells.Add(map[x - 1, y]);
                }
                if (y <= border) isOnEdge = true;
                if (y > border && map[x, y] != map[x, y - 1])
                {
                    isBorder = true;
                    if (!AdjacentCells.Contains(map[x, y - 1])) AdjacentCells.Add(map[x, y - 1]);
                }
                if (x >= map.GetLength(0) - 1 - border) isOnEdge = true;
                if (x < map.GetLength(0) - 1 - border && map[x, y] != map[x + 1, y])
                {
                    isBorder = true;
                    if (!AdjacentCells.Contains(map[x + 1, y])) AdjacentCells.Add(map[x + 1, y]);
                }
                if (y >= map.GetLength(1) - 1 - border) isOnEdge = true;
                if (y < map.GetLength(1) - 1 - border && map[x, y] != map[x, y + 1])
                {
                    isBorder = true;
                    if (!AdjacentCells.Contains(map[x, y + 1])) AdjacentCells.Add(map[x, y + 1]);
                }
                if (isBorder) BorderCells.Add(point);
            }
        }
    }
}
