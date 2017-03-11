//
//  ViewController.m
//  Quadcopter Controller
//
//  Created by Yongyang Nie on 3/4/17.
//  Copyright © 2017 Yongyang Nie. All rights reserved.
//

#import "ViewController.h"

@interface ViewController ()

@end

@implementation ViewController

-(void)rightPan:(UIPanGestureRecognizer *)pan{
    
}

-(void)leftPan:(UIPanGestureRecognizer *)pan{
    
}

- (void)viewDidLoad {
    [super viewDidLoad];
    
    UIPanGestureRecognizer *rightPan = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(rightPan:)];
    UIPanGestureRecognizer *leftPan = [[UIPanGestureRecognizer alloc] initWithTarget:self action:@selector(leftPan:)];
    
    [self.leftBase addGestureRecognizer:leftPan];
    [self.rightBase addGestureRecognizer:rightPan];
    // Do any additional setup after loading the view, typically from a nib.
}


- (void)didReceiveMemoryWarning {
    [super didReceiveMemoryWarning];
    // Dispose of any resources that can be recreated.
}


@end
