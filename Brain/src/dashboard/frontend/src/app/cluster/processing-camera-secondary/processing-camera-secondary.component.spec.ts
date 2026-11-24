import { ComponentFixture, TestBed } from '@angular/core/testing';

import { ProcessingCameraSecondaryComponent } from './processing-camera-secondary.component';

describe('ProcessingCameraSecondaryComponent', () => {
  let component: ProcessingCameraSecondaryComponent;
  let fixture: ComponentFixture<ProcessingCameraSecondaryComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [ProcessingCameraSecondaryComponent]
    })
    .compileComponents();
    
    fixture = TestBed.createComponent(ProcessingCameraSecondaryComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
