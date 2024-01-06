package frc.robot;

public class ShooterTable {
    static ShooterTable instance = new ShooterTable();
    public static ShooterTable getInstance()
    {
        return instance;
    }

    public InterpolatingTree shooterTable = new InterpolatingTree();
    public InterpolatingTree hoodTable = new InterpolatingTree();

    public ShooterTable()
    {
        {
            shooterTable.put(6.0, 1900.0);
            shooterTable.put(69.45, 2000.0);
            shooterTable.put(84.77, 2030.0); // 2050
            shooterTable.put(94.47, 2050.0);
            shooterTable.put(135.5, 2300.0); // 2450

            shooterTable.put(202.0, 2700.0);
            shooterTable.put(241.78, 3000.0);
            shooterTable.put(291.0, 3100.0);
        }
        {
            hoodTable.put(6.0, 0.36);
            hoodTable.put(69.45, 0.4);//.45
            hoodTable.put(84.77, 0.4);
            hoodTable.put(94.47, 0.45);
            hoodTable.put(135.5, 0.45); //0.49
            hoodTable.put(202.0, 0.5);
            hoodTable.put(241.78, 0.55);
            hoodTable.put(291.0, 0.5);
        }
    }
}
