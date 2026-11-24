import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ProcessingCameraPrimaryComponent } from './processing-camera-primary.component';

describe('ProcessingCameraPrimaryComponent', () => {
  let component: ProcessingCameraPrimaryComponent;
  let fixture: ComponentFixture<ProcessingCameraPrimaryComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [ProcessingCameraPrimaryComponent]
    })
    .compileComponents();
    
    fixture = TestBed.createComponent(ProcessingCameraPrimaryComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
