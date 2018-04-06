package org.firstinspires.ftc.teamcode.util;

public interface Producer<T> {
    T get() throws InterruptedException;
}
